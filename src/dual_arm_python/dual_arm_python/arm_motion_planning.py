#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    WorkspaceParameters,
)

class ArmMotionPlanning(Node):
    def __init__(self):
        super().__init__("arm_motion_planning")
        self.get_logger().info("Arm Motion Planning Node Initialized")

        # Load calibration data
        self.calib_file = "/home/rosi/.ros2/easy_handeye2/calibrations/zed_left_arm_calib.calib"
        self.T_base_cam = self.load_calib(self.calib_file)
        self.get_logger().info("Calibration loaded ✓")

        # # Subscribe to ArUco pose topic
        # self.sub = self.create_subscription(
        #     PoseStamped,
        #     "/aruco_single/pose",
        #     self.pose_cb,
        #     10,
        # )

        # Initialize MoveItPy for dual-arm motion planning
        self.moveit = MoveItPy(node_name="dual_arm_moveit")
        self.left_arm = self.moveit.get_planning_component("left_arm")
        self.right_arm = self.moveit.get_planning_component("right_arm")

    def load_calib(self, path: str):
        with open(path) as f:
            data = yaml.safe_load(f)
        t = data["transform"]["translation"]
        q = data["transform"]["rotation"]
        T = np.eye(4)
        T[:3, 3] = [t["x"], t["y"], t["z"]]
        T[:3, :3] = R.from_quat([q["x"], q["y"], q["z"], q["w"]]).as_matrix()
        return T

    # Handle the pose message
    def create_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0, frame="base_link"):
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def right_move_to_pose(self, pose: PoseStamped):
        self.right_arm.set_goal_state(pose_stamped_msg=pose, pose_link="tool0")
        plan = self.right_arm.plan()
        if plan:
            self.right_arm.execute(plan)
            self.get_logger().info("Right arm moved to target pose.")
        else:
            self.get_logger().error("Failed to plan for right arm.")

    def left_move_to_pose(self, pose: PoseStamped):
        self.left_arm.set_goal_state(pose_stamped_msg=pose, pose_link="tool0")
        plan = self.left_arm.plan()
        if plan:
            self.left_arm.execute(plan)
            self.get_logger().info("Left arm moved to target pose.")
        else:
            self.get_logger().error("Failed to plan for left arm.")

    def dual_move_to_pose(self, left_pose: PoseStamped, right_pose: PoseStamped):
        self.left_arm.set_goal_state(pose_stamped_msg=left_pose, pose_link="tool0")
        self.right_arm.set_goal_state(pose_stamped_msg=right_pose, pose_link="tool0")
        plan = self.moveit.plan()
        if plan:
            self.moveit.execute(plan)
            self.get_logger().info("Both arms moved to target poses.")
        else:
            self.get_logger().error("Failed to plan for dual-arm motion.")
