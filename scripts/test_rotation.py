import rclpy
import math
import time
import tf2_ros
from rclpy.node import Node
from aruco_moveit_planner.moveit_planner import MoveItPlanOnlyClient
from moveit_msgs.srv import GetPositionFK
from std_msgs.msg import String
from scipy.spatial.transform import Rotation as R


# ---- helper functions (take `planner` explicitly) ----
class URNode(Node):
    def __init__(self):
        super().__init__('ur_rotate_node')
        self._urscript_pub = self.create_publisher(
            String, '/urscript_interface/script_command', 10)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

def rotate_tool_x(planner, theta_rad, accel=0.5, vel=0.2):
    script = String()
    script.data = (
        "def rot_x():\n"
        f"  rot = p[0, 0, 0, {theta_rad}, 0, 0]\n"
        "  target = pose_trans(get_actual_tcp_pose(), rot)\n"
        f"  movel(target, a={accel}, v={vel})\n"
        "end\n"
    )
    planner._urscript_pub.publish(script)
    planner.get_logger().info(f"Sent tool-X rotation: {theta_rad} rad")


def orientation_from_joints(planner, joint_names, joint_positions,
                            base='base', eef='tool0'):
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


def current_orientation(planner, base='base', eef='tool0'):
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


def main():
    rclpy.init()
    ur_node = URNode()
    print(">>> planner created") 

    # attach the publisher + FK client to `planner`
    ur_node._urscript_pub = ur_node.create_publisher(
        String, '/urscript_interface/script_command', 10)
    ur_node._fk_client = ur_node.create_client(GetPositionFK, "/compute_fk")
    ur_node._fk_client.wait_for_service(timeout_sec=5.0)

    time.sleep(1.0)   # let publisher connect

    # --- example: reorient to match a target joint configuration ---
    joint_names = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
    ]

    target_joint_angles = [-2.006246, -1.900777, -2.201122, 0.582374, -4.237320, -0.100220]   

    # r_tgt = orientation_from_joints(ur_node, joint_names, target_joint_angles)
    r_tgt = R.from_quat([0.090, 0.674, -0.699, -0.220])
    r_cur = current_orientation(ur_node)
    print(f"Current orientation (quaternion): {r_cur.as_quat()}")
    print(f"Target orientation (quaternion): {r_tgt.as_quat()}")
    

    if r_tgt is not None and r_cur is not None:
        rotate_to_match(ur_node, r_cur, r_tgt)
        rclpy.spin_once(ur_node, timeout_sec=1.0)   # let script go out
    else:
        ur_node.get_logger().error("Could not get current or target orientation.")

    ur_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()