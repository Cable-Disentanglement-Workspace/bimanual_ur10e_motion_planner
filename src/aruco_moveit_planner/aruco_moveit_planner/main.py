"""Entry point: read ArUco pose → transform → request MoveIt plan-only trajectory.

The script orchestrates three independent modules in sequence:

1. **pose_reader**      — acquire the ArUco marker ``PoseStamped`` (topic or JSON).
2. **frame_transformer** — apply hand-eye calibration; compute ``left_tcp`` goal.
3. **moveit_planner**   — send a plan-only request to move_group; RViz shows result.

Prerequisites
-------------
``dual_arm_real_v2.launch.py`` must already be
running so that move_group and RViz are active with both arms visible.

Usage::

    # Read from live topic (default)
    ros2 run aruco_moveit_planner plan_to_aruco

    # Read from JSON file
    ros2 run aruco_moveit_planner plan_to_aruco \\
        --source json \\
        --json-path /path/to/aruco_pose.json

    # Override calibration path or topic
    ros2 run aruco_moveit_planner plan_to_aruco \\
        --calib-path /custom/path/to/calib.calib \\
        --topic /my_aruco/pose \\
        --timeout 15.0

Via launch file::

    ros2 launch aruco_moveit_planner plan_to_aruco.launch.py source:=json \\
        json_path:=/path/to/aruco_pose.json
"""

import argparse
import os
import sys

from ament_index_python.packages import get_package_share_directory
import rclpy

from aruco_moveit_planner.frame_transformer import (
    compute_tcp_goal,
    load_calibration,
    log_transform_summary,
    transform_marker_to_base,
)
from aruco_moveit_planner.moveit_planner import MoveItPlanOnlyClient
from aruco_moveit_planner.pose_reader import PoseTopicReader, read_from_json

# ── Defaults ──────────────────────────────────────────────────────────────────

_DEFAULT_CALIB = os.path.join(
    get_package_share_directory("aruco_moveit_planner"),
    "calibrations",
    "zed_left_arm_calib.calib",
)
_DEFAULT_TOPIC = "/aruco_single/pose"


# ── Argument parsing ──────────────────────────────────────────────────────────


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="plan_to_aruco",
        description=(
            "Plan a MoveIt 2 trajectory for the left UR10e arm to reach an "
            "ArUco marker detected by the static ZED Mini camera. "
            "Visualises the plan in RViz — does NOT execute on real hardware."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "--source",
        choices=["topic", "json"],
        default="topic",
        help="Where to read the ArUco marker pose from.",
    )
    p.add_argument(
        "--json-path",
        default="",
        metavar="PATH",
        help="Path to the PoseStamped JSON file (required when --source json).",
    )
    p.add_argument(
        "--calib-path",
        default=_DEFAULT_CALIB,
        metavar="PATH",
        help="Path to the easy_handeye2 .calib file.",
    )
    p.add_argument(
        "--topic",
        default=_DEFAULT_TOPIC,
        metavar="TOPIC",
        help="ROS 2 topic that publishes the ArUco PoseStamped.",
    )
    p.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        metavar="SEC",
        help="Seconds to wait for a topic message before aborting.",
    )
    return p


def _strip_ros_args(argv: list) -> list:
    """Remove ROS 2 argument block (``--ros-args`` and everything after).

    Args:
        argv: Raw ``sys.argv[1:]``.

    Returns:
        Argument list with the ROS 2 block removed.
    """
    if "--ros-args" in argv:
        return argv[: argv.index("--ros-args")]
    return argv


# ── Orchestration ─────────────────────────────────────────────────────────────


def main(argv=None) -> None:
    """Orchestrate pose reading, frame transformation, and MoveIt planning.

    Args:
        argv: Argument list.  Defaults to ``sys.argv[1:]`` when ``None``.
    """
    raw_argv = sys.argv[1:] if argv is None else argv
    args = _build_parser().parse_args(_strip_ros_args(raw_argv))

    # ── 1. Load hand-eye calibration (no ROS node needed) ────────────────────
    try:
        T_base_camera = load_calibration(args.calib_path)
    except (FileNotFoundError, KeyError) as exc:
        print(f"[ERROR] Calibration: {exc}", file=sys.stderr)
        sys.exit(1)

    rclpy.init()

    # ── 2. Acquire ArUco marker pose ──────────────────────────────────────────
    try:
        if args.source == "json":
            if not args.json_path:
                print(
                    "[ERROR] --json-path is required when --source json.",
                    file=sys.stderr,
                )
                rclpy.shutdown()
                sys.exit(1)
            try:
                marker_pose_camera = read_from_json(args.json_path)
            except (FileNotFoundError, KeyError) as exc:
                print(f"[ERROR] JSON read: {exc}", file=sys.stderr)
                rclpy.shutdown()
                sys.exit(1)
        else:
            reader = PoseTopicReader(topic=args.topic, timeout_sec=args.timeout)
            try:
                marker_pose_camera = reader.wait_for_pose()
            except TimeoutError as exc:
                print(f"[ERROR] Topic read: {exc}", file=sys.stderr)
                sys.exit(1)
            finally:
                reader.destroy_node()

    except Exception as exc:  # pragma: no cover — safety net
        print(f"[ERROR] Unexpected error during pose acquisition: {exc}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    # ── 3. Transform to robot base frame; compute left_tcp goal ───────────────
    marker_pose_base = transform_marker_to_base(T_base_camera, marker_pose_camera)
    tcp_goal = compute_tcp_goal(marker_pose_base)

    # ── 4. Plan-only MoveIt request ───────────────────────────────────────────
    try:
        planner = MoveItPlanOnlyClient()
    except RuntimeError as exc:
        print(f"[ERROR] MoveIt planner init: {exc}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    # Log the full transform chain for debugging.
    log_transform_summary(
        marker_pose_camera, marker_pose_base, tcp_goal, planner.get_logger()
    )

    success = planner.plan_to_pose(tcp_goal)

    if success:
        # Re-publish the planned trajectory on a loop so RViz animates it
        # continuously — no "Loop Animation" checkbox needed in RViz.
        planner.start_display_loop()
        try:
            rclpy.spin(planner)
        except KeyboardInterrupt:
            pass

    planner.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
