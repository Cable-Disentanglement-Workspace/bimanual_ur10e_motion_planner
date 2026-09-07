import argparse
import rclpy
import time
import tf2_ros
import glob
import yaml
from aruco_moveit_planner.moveit_planner import MoveItPlanOnlyClient
from geometry_msgs.msg import PoseStamped, Point
from rosidl_runtime_py import message_to_ordereddict, set_message_fields
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.srv import GetPositionFK
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String
from scipy.spatial.transform import Rotation as R


SAVE_DIR = "/home/rosi/ZD/zed_Motion/src/scripts/trajectories"
EEF_LINK = "right_tcp"
BASE_FRAME = "right_base"
LABEL = "OMPL"


def move_with_retry(planner, target, arm="left", max_retries=5,
                    execute=False, auto_execute=False, constrain_joints=True,
                    pipeline_id="ompl", planner_id="RRTstarkConfigDefault"):
    for attempt in range(max_retries):
        if arm == "left":
            success = planner.plan_to_pose(target)
        else:
            success = planner.plan_to_pose2(target, constrain_joints=constrain_joints)

        if success:
            print(f"Plan succeeded on attempt {attempt + 1}")

            if execute:
                if auto_execute:
                    planner.execute_stored_trajectory()
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


def rotate_to_match(planner, r_cur, r_tgt, accel=0.5, vel=0.2):
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


def main(args):
    rclpy.init()

    planner = MoveItPlanOnlyClient()

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
    right_target.pose.orientation.x = 0.090
    right_target.pose.orientation.y = 0.674
    right_target.pose.orientation.z = -0.699
    right_target.pose.orientation.w = -0.220

    if args.r1_joints is not None:
        robot1_joints = args.r1_joints    # [j0, j1, j2, j3, j4, j5]

    if args.r2_joints is not None:
        robot2_joints = args.r2_joints    # [j0, j1, j2, j3, j4, j5]

    right_trainsit = PoseStamped()
    right_trainsit.header.frame_id = "right_base"
    right_trainsit.pose.position.x = right_target.pose.position.x + 0.15
    right_trainsit.pose.position.y = right_target.pose.position.y + 0.15
    right_trainsit.pose.position.z = right_target.pose.position.z
    right_trainsit.pose.orientation.x = 0.090
    right_trainsit.pose.orientation.y = 0.674
    right_trainsit.pose.orientation.z = -0.699
    right_trainsit.pose.orientation.w = -0.220

    joint_names = [
    'right_shoulder_pan_joint', 'right_shoulder_lift_joint', 'right_elbow_joint',
    'right_wrist_1_joint', 'right_wrist_2_joint', 'right_wrist_3_joint',
    ]

    target_joint_angles = robot1_joints
    r_tgt = orientation_from_joints(planner, joint_names, target_joint_angles)
    r_cur = current_orientation(planner)

    planner._add_box_obstacle("wall1", -0.4, right_target.pose.position.y + 0.1,
                              0.5, 0.2, 0.01, 0.8, frame="right_base")
    time.sleep(1.0)

    ok = move_with_retry(planner, right_trainsit, arm="right",
                         execute=True, auto_execute=args.auto_execute)

    if ok:
        planner._remove_obstacle("wall1")
        ok = move_with_retry(planner, right_target, arm="right",
                             execute=True, auto_execute=args.auto_execute)
        time.sleep(1.0)
        if r_tgt is not None and r_cur is not None:
            rotate_to_match(planner, r_cur, r_tgt)
            rclpy.spin_once(planner, timeout_sec=1.0)   # let script go out
        else:
            planner.get_logger().error("Could not get current or target orientation.")

    print_tcp_pose(planner)
    planner.destroy_node()
    rclpy.shutdown()


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
    parser.add_argument("--auto-execute", action="store_true")
    args = parser.parse_args()
    main(args)