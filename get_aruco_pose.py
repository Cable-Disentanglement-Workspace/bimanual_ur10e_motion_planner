#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped


CALIB_FILE = "/home/rosi/.ros2/easy_handeye2/calibrations/zed_left_arm_calib.calib"


def load_calib(path: str):
    with open(path) as f:
        data = yaml.safe_load(f)
    t = data["transform"]["translation"]
    q = data["transform"]["rotation"]
    T = np.eye(4)
    T[:3, 3] = [t["x"], t["y"], t["z"]]
    T[:3, :3] = R.from_quat([q["x"], q["y"], q["z"], q["w"]]).as_matrix()
    return T


class ArucoPoseNode(Node):
    def __init__(self):
        super().__init__("aruco_pose_node")
        self.T_base_cam = load_calib(CALIB_FILE)
        self.get_logger().info("Calibration loaded ✓")

        self.sub = self.create_subscription(
            PoseStamped,
            "/aruco_single/pose",
            self.pose_cb,
            10,
        )

    def pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation

        T_cam_marker = np.eye(4)
        T_cam_marker[:3, 3] = [p.x, p.y, p.z]
        T_cam_marker[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        T_base_marker = self.T_base_cam @ T_cam_marker

        pos = T_base_marker[:3, 3]
        rot = R.from_matrix(T_base_marker[:3, :3])
        euler_deg = rot.as_euler("xyz", degrees=True)
        euler_rad = rot.as_euler("xyz", degrees=False)

        print("\n" + "="*60)
        print("  ArUco pose in ROBOT BASE FRAME")
        print("="*60)
        print(f"  X : {pos[0]*1000:+8.2f} mm  ({pos[0]:.6f} m)")
        print(f"  Y : {pos[1]*1000:+8.2f} mm  ({pos[1]:.6f} m)")
        print(f"  Z : {pos[2]*1000:+8.2f} mm  ({pos[2]:.6f} m)")
        print(f"  Rx: {euler_deg[0]:+8.3f} deg  ({euler_rad[0]:+.6f} rad)")
        print(f"  Ry: {euler_deg[1]:+8.3f} deg  ({euler_rad[1]:+.6f} rad)")
        print(f"  Rz: {euler_deg[2]:+8.3f} deg  ({euler_rad[2]:+.6f} rad)")
        print("="*60)

        self.destroy_subscription(self.sub)
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ArucoPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
