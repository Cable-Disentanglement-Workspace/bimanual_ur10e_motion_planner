import argparse
import rclpy
import sys
import time
import tf2_ros
import glob
import yaml
from motion_completion import (
    BASE_MOVEIT_VELOCITY_SCALE,
    BASE_URSCRIPT_VELOCITY,
    COMPLETION_TOKEN,
    URScriptCompletionServer,
    local_ip_for_peer,
    scaled_motion_values,
    validated_motion_values,
)
from aruco_moveit_planner.moveit_planner import MoveItPlanOnlyClient
from geometry_msgs.msg import PoseStamped, Point
from rosidl_runtime_py import message_to_ordereddict, set_message_fields
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.srv import GetPositionFK
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import ColorRGBA, String
from scipy.spatial.transform import Rotation as R


SAVE_DIR = "/home/rosi/ZD/zed_Motion/src/scripts/trajectories"
EEF_LINK = "right_tcp"
BASE_FRAME = "right_base"
LABEL = "OMPL"


def move_with_retry(planner, target, arm="left", max_retries=5,
                    execute=False, auto_execute=False, constrain_joints=True,
                    pipeline_id="ompl", planner_id="RRTstarkConfigDefault",
                    execution_timeout=120.0):
    for attempt in range(max_retries):
        t0 = time.perf_counter()
        if arm == "left":
            success = planner.plan_to_pose(target)
        else:
            success = planner.plan_to_pose2(target, constrain_joints=constrain_joints,
                                            pipeline_id=pipeline_id, planner_id=planner_id)
        print(f"[{planner_id}] planning time: {time.perf_counter() - t0:.2f} s "
              f"({'success' if success else 'failed'})")

        if success:
            print(f"Plan succeeded on attempt {attempt + 1}")

            if execute:
                if auto_execute:
                    if not planner.execute_stored_trajectory(
                        timeout_sec=float(execution_timeout)
                    ):
                        print("Trajectory execution failed; stopping without retry.")
                        return False
                else:
                    print("Plan stored, not executing (no --auto-execute).")
            return True

        print(f"Attempt {attempt + 1}/{max_retries} failed, retrying...")

    print(f"Failed after {max_retries} attempts!")
    return False


def print_tcp_pose(planner):
    tf_buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buf, planner)
    deadline = planner.get_clock().now() + rclpy.duration.Duration(seconds=2.0)
    while planner.get_clock().now() < deadline:
        try:
            t = tf_buf.lookup_transform("right_base", "right_tcp", rclpy.time.Time())
            p, r = t.transform.translation, t.transform.rotation
            print(f"right_tcp pos=({p.x:.4f},{p.y:.4f},{p.z:.4f}) "
                  f"quat=({r.x:.4f},{r.y:.4f},{r.z:.4f},{r.w:.4f})")
            return
        except Exception:
            rclpy.spin_once(planner, timeout_sec=0.1)
    print("right_tcp: TF unavailable")


def plan_with(planner, target, pipeline_id, planner_id, arm="right"):
    """Run one plan with a specific pipeline/planner. Returns RobotTrajectory or None."""
    if arm == "left":
        ok = planner.plan_to_pose(target, pipeline_id=pipeline_id, planner_id=planner_id)
    else:
        ok = planner.plan_to_pose2(target, pipeline_id=pipeline_id, planner_id=planner_id)
    return planner._stored_trajectory if ok else None


def orientation_from_joints(planner, joint_names, joint_positions,
                            base='right_base', eef='right_tcp'):
    """Compute TCP orientation from joint angles. No motion — pure calculation."""
    req = GetPositionFK.Request()
    req.header.frame_id = base
    req.fk_link_names = [eef]
    req.robot_state.joint_state.name = joint_names
    req.robot_state.joint_state.position = list(joint_positions)

    fut = planner._fk_client.call_async(req)
    rclpy.spin_until_future_complete(planner, fut)
    res = fut.result()

    if res and res.error_code.val == 1 and res.pose_stamped:
        q = res.pose_stamped[0].pose.orientation
        return R.from_quat([q.x, q.y, q.z, q.w])
    return None


def current_orientation(planner, base='right_base', eef='right_tcp'):
    """Read current TCP orientation from TF. No motion."""
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, planner)
    deadline = planner.get_clock().now() + rclpy.duration.Duration(seconds=2.0)
    while planner.get_clock().now() < deadline:
        try:
            t = buf.lookup_transform(base, eef, rclpy.time.Time())
            q = t.transform.rotation
            return R.from_quat([q.x, q.y, q.z, q.w])
        except Exception:
            rclpy.spin_once(planner, timeout_sec=0.1)
    return None


def rotate_to_match(planner, r_cur, r_tgt, accel=0.05, vel=0.02):
    """Rotate TCP in place so its orientation matches r_tgt. r_cur, r_tgt are scipy Rotations."""
    r_rel = r_cur.inv() * r_tgt              # relative rotation in tool frame
    rotvec = r_rel.as_rotvec()               # [rx, ry, rz] for pose_trans

    script = String()
    script.data = (
        "def rot_match():\n"
        f"  rot = p[0, 0, 0, {rotvec[0]}, {rotvec[1]}, {rotvec[2]}]\n"
        "  target = pose_trans(get_actual_tcp_pose(), rot)\n"
        f"  movel(target, a={accel}, v={vel})\n"
        "end\n"
    )
    planner._urscript_pub.publish(script)
    planner.get_logger().info(f"Rotating to match, rotvec={rotvec}")


def rotate_to_match_qnear(
    planner,
    target_joints,
    target_position,
    robot_ip,
    accel=0.1,
    vel=0.1,
    motion_timeout=180.0,
):
    """Keep current TCP x,y,z and move joints close to target_joints.

    Gets FK of target_joints to obtain its orientation, builds a hybrid pose
    (current position + target orientation), then solves IK seeded with
    target_joints so the result is as close as possible to the target
    configuration while keeping the same Cartesian position.

    target_joints: list/tuple of 6 joint angles in radians.
    """
    q = target_joints
    target_x, target_y, target_z = target_position
    fmt = lambda joints: [round(value, 4) for value in joints]
    planner.get_logger().info(
        f"rotate_to_match_qnear target_joints={fmt(list(q))}"
    )

    try:
        callback_ip = local_ip_for_peer(robot_ip)
        with URScriptCompletionServer(callback_ip) as completion:
            script = String()
            script.data = (
                "def rot_match_qnear():\n"
                f"  ack_open = socket_open(\"{completion.host}\", "
                f"{completion.port}, \"motion_ack\")\n"
                "  if ack_open:\n"
                "    cur_pose = get_actual_tcp_pose()\n"
                f"    qnear = [{q[0]}, {q[1]}, {q[2]}, {q[3]}, {q[4]}, {q[5]}]\n"
                "    tgt_pose = get_forward_kin(qnear)\n"
                f"    hybrid = p[{target_x}, {target_y}, {target_z}, "
                "tgt_pose[3], tgt_pose[4], tgt_pose[5]]\n"
                "    q_target = get_inverse_kin(hybrid, qnear=qnear)\n"
                "    textmsg(\"q_cur   =\", get_actual_joint_positions())\n"
                "    textmsg(\"q_target=\", q_target)\n"
                f"    movej(q_target, a={accel}, v={vel})\n"
                f"    socket_send_string(\"{COMPLETION_TOKEN.decode('ascii')}\", "
                "\"motion_ack\")\n"
                "    socket_close(\"motion_ack\")\n"
                "  else:\n"
                "    textmsg(\"Motion acknowledgement socket failed; move cancelled\")\n"
                "  end\n"
                "end\n"
            )
            planner.get_logger().info(
                "Starting final URScript alignment; waiting for robot-side "
                f"completion acknowledgement on {completion.host}:{completion.port}..."
            )
            planner._urscript_pub.publish(script)
            completed = completion.wait(motion_timeout)
    except OSError as exc:
        planner.get_logger().error(
            f"Could not create URScript completion channel: {exc}"
        )
        return False

    if not completed:
        planner.get_logger().error(
            "Final URScript alignment did not acknowledge completion within "
            f"{float(motion_timeout):.1f} seconds."
        )
        return False

    planner.get_logger().info(
        "Final URScript alignment complete; robot-side movej returned."
    )
    return True


def main(args):
    direct_values = (
        args.moveit_velocity_scaling,
        args.moveit_acceleration_scaling,
        args.urscript_velocity,
        args.urscript_acceleration,
    )
    if all(value is None for value in direct_values):
        dynamics = scaled_motion_values(
            args.speed_multiplier, args.acceleration_multiplier
        )
    elif any(value is None for value in direct_values):
        raise ValueError(
            "Explicit motion settings require all four of "
            "--moveit-velocity-scaling, --moveit-acceleration-scaling, "
            "--urscript-velocity, and --urscript-acceleration"
        )
    else:
        dynamics = validated_motion_values(*direct_values)
    rclpy.init()
    moveit_velocity = dynamics["moveit_velocity"]
    moveit_acceleration = dynamics["moveit_acceleration"]
    urscript_velocity = dynamics["urscript_velocity"]
    urscript_acceleration = dynamics["urscript_acceleration"]
    execution_timeout = max(
        120.0,
        120.0 * BASE_MOVEIT_VELOCITY_SCALE / moveit_velocity,
    )
    urscript_timeout = max(
        180.0,
        180.0 * BASE_URSCRIPT_VELOCITY / urscript_velocity,
    )
    planner = MoveItPlanOnlyClient(
        velocity_scaling=moveit_velocity,
        acceleration_scaling=moveit_acceleration,
    )
    planner.get_logger().info(
        "Motion dynamics: "
        f"MoveIt velocity scaling={moveit_velocity:.3f}, "
        f"MoveIt acceleration scaling={moveit_acceleration:.3f}, "
        f"URScript v={urscript_velocity:.3f} rad/s, "
        f"URScript a={urscript_acceleration:.3f} rad/s², "
        f"execution timeout={execution_timeout:.0f}s, "
        f"URScript timeout={urscript_timeout:.0f}s"
    )

    from visualization_msgs.msg import MarkerArray as _MA
    planner._marker_pub = planner.create_publisher(_MA, "/trajectory_comparison", 1)
    planner._fk_client = planner.create_client(GetPositionFK, "/compute_fk")
    planner._fk_client.wait_for_service(timeout_sec=5.0)
    planner._urscript_pub = planner.create_publisher(
        String, '/right_urscript_interface/script_command', 10)

    right_target = PoseStamped()
    right_target.header.frame_id = "right_base"
    right_target.pose.position.x = args.x
    right_target.pose.position.y = args.y
    right_target.pose.position.z = args.z
    right_target.pose.orientation.x = args.rx
    right_target.pose.orientation.y = args.ry
    right_target.pose.orientation.z = args.rz
    right_target.pose.orientation.w = args.rw

    robot1_joints = None
    robot2_joints = None
    if args.r1_joints is not None:
        robot1_joints = args.r1_joints    # [j0, j1, j2, j3, j4, j5]

    if args.r2_joints is not None:
        robot2_joints = args.r2_joints    # [j0, j1, j2, j3, j4, j5]

    right_trainsit = PoseStamped()
    right_trainsit.header.frame_id = "right_base"
    right_trainsit.pose.position.x = right_target.pose.position.x + 0.15
    right_trainsit.pose.position.y = right_target.pose.position.y + 0.15
    right_trainsit.pose.position.z = right_target.pose.position.z + 0.05
    right_trainsit.pose.orientation.x = args.rx
    right_trainsit.pose.orientation.y = args.ry
    right_trainsit.pose.orientation.z = args.rz
    right_trainsit.pose.orientation.w = args.rw

    joint_names = [
    'right_shoulder_pan_joint', 'right_shoulder_lift_joint', 'right_elbow_joint',
    'right_wrist_1_joint', 'right_wrist_2_joint', 'right_wrist_3_joint',
    ]

    target_joint_angles = robot2_joints
    if target_joint_angles is None:
        planner.get_logger().error(
            "Robot2 checkpoint start joints are required (--r2-joints)."
        )
        planner.destroy_node()
        rclpy.shutdown()
        return False

    planner._add_box_obstacle("wall1", -0.4, right_target.pose.position.y + 0.1,
                              0.5, 0.2, 0.01, 0.8, frame="right_base")
    time.sleep(1.0)

    ok = move_with_retry(planner, right_trainsit, arm="right",
                         execute=True, auto_execute=args.auto_execute,
                         execution_timeout=execution_timeout)

    if ok:
        planner._remove_obstacle("wall1")
        # Pilz LIN: straight line in Cartesian space. Joint constraints off —
        # Pilz rejects goals that mix joint and pose constraints.
        ok = move_with_retry(planner, right_target, arm="right",
                             execute=True, auto_execute=args.auto_execute,
                             execution_timeout=execution_timeout,
                             constrain_joints=False,
                             pipeline_id="pilz_industrial_motion_planner",
                             planner_id="LIN")
        time.sleep(1.0)
        if target_joint_angles is not None:
            ok = rotate_to_match_qnear(
                planner,
                target_joint_angles,
                (args.x, args.y, args.z),
                args.right_robot_ip,
                accel=urscript_acceleration,
                vel=urscript_velocity,
                motion_timeout=urscript_timeout,
            )
        else:
            planner.get_logger().error("No target joint angles provided (--r2-joints).")

    if not ok:
        planner.get_logger().error(
            "Motion planning or execution did not complete successfully."
        )

    print_tcp_pose(planner)
    planner.destroy_node()
    rclpy.shutdown()
    return bool(ok)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--x",  type=float, default=-0.317)
    parser.add_argument("--y",  type=float, default=-0.708)
    parser.add_argument("--z",  type=float, default=0.367)
    parser.add_argument("--rx", type=float, default=0.315)
    parser.add_argument("--ry", type=float, default=0.603)
    parser.add_argument("--rz", type=float, default=-0.582)
    parser.add_argument("--rw", type=float, default=-0.446)
    parser.add_argument("--r1-joints", type=float, nargs=6, default=None,
                    help="Robot1 median joint angles (rad)")
    parser.add_argument("--r2-joints", type=float, nargs=6, default=None,
                    help="Robot2 median joint angles (rad)")
    parser.add_argument("--right-robot-ip", default="192.168.1.20",
                    help="Robot executing the final URScript alignment")
    parser.add_argument("--speed-multiplier", type=float, default=1.0,
                    help="Legacy linked velocity multiplier")
    parser.add_argument("--acceleration-multiplier", type=float, default=1.0,
                    help="Legacy linked acceleration multiplier")
    parser.add_argument("--moveit-velocity-scaling", type=float, default=None,
                    help="Explicit MoveIt velocity scaling in the range (0, 1]")
    parser.add_argument("--moveit-acceleration-scaling", type=float, default=None,
                    help="Explicit MoveIt acceleration scaling in the range (0, 1]")
    parser.add_argument("--urscript-velocity", type=float, default=None,
                    help="Explicit final movej joint velocity in rad/s")
    parser.add_argument("--urscript-acceleration", type=float, default=None,
                    help="Explicit final movej joint acceleration in rad/s²")
    parser.add_argument("--auto-execute", action="store_true")
    args = parser.parse_args()
    sys.exit(0 if main(args) else 1)
