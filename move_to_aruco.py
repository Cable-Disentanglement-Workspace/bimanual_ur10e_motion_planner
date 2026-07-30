#!/usr/bin/env python3
"""
move_to_aruco.py

One-shot script: subscribes to the ArUco marker pose from aruco_ros
(topic /aruco_single/pose, published in the zed_left_camera_frame),
transforms it into the `left_base` frame using the hand-eye calibration
read DIRECTLY from the .calib YAML file (no TF lookup needed for the
camera->base transform -- this avoids ROS2 static-TF late-joiner issues).

The end-effector is positioned PERPENDICULAR to the marker's own face
(using the marker's own normal direction, not the current end-effector
position), offset STANDOFF_DISTANCE meters back from the marker along
that normal -- i.e. squared-up and ready to approach/grab the marker
head-on, like a gripper lining up on a flat surface.

Then sends a single MoveGroup goal to move the `left_arm` planning
group's end-effector to that computed pose.

Usage:
    python3 move_to_aruco.py              # plans AND executes on the robot
    python3 move_to_aruco.py --dry-run    # only computes and prints the pose

Requires (already running in separate terminals):
    - dual_arm_moveit_config dual_arm_real_v2.launch.py   (robot + move_group)
    - zed_wrapper zed_camera.launch.py camera_model:=zedm  (camera)
    - aruco_ros single ...                                 (marker detection,
      publishing geometry_msgs/PoseStamped on /aruco_single/pose)

Does NOT require easy_handeye2 publish.launch.py to be running, since the
calibration is read directly from disk.
"""

import os
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    WorkspaceParameters,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped


# ---- Configuration -----------------------------------------------------
CALIBRATION_FILE = os.path.expanduser(
    "~/.ros2/easy_handeye2/calibrations/zed_left_arm_calib.calib"
)
ROBOT_BASE_FRAME = "left_base"      # planning frame for the left_arm group
ARUCO_POSE_TOPIC = "/aruco_single/pose"  # published directly by aruco_ros
PLANNING_GROUP = "left_arm"
END_EFFECTOR_LINK = "left_tool0"
PLANNING_TIME = 5.0
POSITION_TOLERANCE = 0.005   # meters
ORIENTATION_TOLERANCE = 0.01  # radians
POSE_WAIT_TIMEOUT_SEC = 10.0

# If True, flip the marker's normal direction before computing the
# approach. The marker's own +Z axis points toward whoever/whatever is
# looking at it (here, the camera). The gripper should stand on the
# SAME side as the camera and face the marker head-on, so usually this
# should stay True. If the computed approach ends up on the wrong side
# of the marker, try setting this to False.
FLIP_MARKER_NORMAL = False

# Distance to stand off from the marker along its normal direction, in
# meters. 0.0 = stop exactly at the marker's surface.
STANDOFF_DISTANCE = 0.10
# -------------------------------------------------------------------------


def load_calibration_transform(path: str) -> np.ndarray:
    """Read the easy_handeye2 .calib YAML file and return a 4x4 homogeneous
    transform matrix mapping points from the camera frame to the robot
    base frame (i.e. T_base_camera)."""
    if not os.path.isfile(path):
        raise RuntimeError(f"Calibration file not found: {path}")

    with open(path, "r") as f:
        data = yaml.safe_load(f)

    t = data["transform"]["translation"]
    r = data["transform"]["rotation"]

    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([r["x"], r["y"], r["z"], r["w"]]).as_matrix()
    T[:3, 3] = [t["x"], t["y"], t["z"]]
    return T


def pose_to_matrix(pose: PoseStamped) -> np.ndarray:
    """Convert a PoseStamped into a 4x4 homogeneous transform matrix."""
    p = pose.pose.position
    o = pose.pose.orientation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def matrix_to_pose(T: np.ndarray, frame_id: str, stamp) -> PoseStamped:
    """Convert a 4x4 homogeneous transform matrix into a PoseStamped."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = T[:3, 3]
    qx, qy, qz, qw = Rotation.from_matrix(T[:3, :3]).as_quat()
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def perpendicular_approach_rotation(
    marker_normal: np.ndarray,
    flip: bool = False,
    up: np.ndarray = np.array([0.0, 0.0, 1.0]),
) -> np.ndarray:
    """Compute a 3x3 rotation matrix for the end-effector so its forward
    (+Z) axis points squarely at the marker, anti-parallel to the
    marker's own normal -- i.e. perpendicular / head-on to the marker's
    face, regardless of where the end-effector currently is."""
    normal = marker_normal / np.linalg.norm(marker_normal)
    gripper_z = normal if flip else -normal

    if abs(np.dot(up, gripper_z)) > 0.999:
        up = np.array([0.0, 1.0, 0.0])

    x_axis = np.cross(up, gripper_z)
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = np.cross(gripper_z, x_axis)

    return np.column_stack([x_axis, y_axis, gripper_z])


class MoveToArucoNode(Node):
    def __init__(self, T_base_camera: np.ndarray):
        super().__init__("move_to_aruco")
        self.T_base_camera = T_base_camera

        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")

        self.latest_marker_pose = None
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, ARUCO_POSE_TOPIC, self._pose_callback, qos
        )

    def _pose_callback(self, msg: PoseStamped):
        self.latest_marker_pose = msg

    def wait_for_marker_pose(self) -> PoseStamped:
        """Wait for at least one marker pose message from aruco_ros."""
        self.get_logger().info(f"Waiting for a message on {ARUCO_POSE_TOPIC} ...")

        deadline = self.get_clock().now() + Duration(seconds=POSE_WAIT_TIMEOUT_SEC)
        while rclpy.ok() and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_marker_pose is not None:
                break
        else:
            raise RuntimeError(
                f"Timed out waiting for {ARUCO_POSE_TOPIC}. "
                "Is aruco_ros running and detecting the marker?"
            )

        msg = self.latest_marker_pose
        self.get_logger().info(
            "Marker pose in {}: pos=({:.3f}, {:.3f}, {:.3f})".format(
                msg.header.frame_id,
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            )
        )
        return msg

    def compute_target_pose(self, marker_pose: PoseStamped) -> PoseStamped:
        """Compute the goal pose for the end-effector: positioned
        STANDOFF_DISTANCE meters off the marker's surface along its own
        normal, oriented perpendicular (head-on) to that surface."""
        T_camera_marker = pose_to_matrix(marker_pose)
        T_base_marker = self.T_base_camera @ T_camera_marker
        marker_pos = T_base_marker[:3, 3]
        marker_normal = T_base_marker[:3, 2]  # marker's own Z axis

        R_goal = perpendicular_approach_rotation(
            marker_normal, flip=FLIP_MARKER_NORMAL
        )

        # Stand off along the marker's normal, on the same side the
        # gripper's forward axis is pointing FROM (i.e. opposite to
        # gripper_z, back toward where the camera/approach side is).
        gripper_z = R_goal[:, 2]
        goal_pos = marker_pos - gripper_z * STANDOFF_DISTANCE

        T_goal = np.eye(4)
        T_goal[:3, :3] = R_goal
        T_goal[:3, 3] = goal_pos

        target_pose = matrix_to_pose(
            T_goal, ROBOT_BASE_FRAME, self.get_clock().now().to_msg()
        )

        quat = [
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z,
            target_pose.pose.orientation.w,
        ]
        roll, pitch, yaw = Rotation.from_quat(quat).as_euler("xyz", degrees=False)

        self.get_logger().info(
            "Marker pose in {}: pos=({:.3f}, {:.3f}, {:.3f})".format(
                ROBOT_BASE_FRAME, marker_pos[0], marker_pos[1], marker_pos[2]
            )
        )
        self.get_logger().info(
            "Marker normal in {}: ({:.3f}, {:.3f}, {:.3f})".format(
                ROBOT_BASE_FRAME, marker_normal[0], marker_normal[1], marker_normal[2]
            )
        )
        self.get_logger().info(
            "Goal pose (standoff={:.2f}m, perpendicular) in {}: "
            "pos=({:.3f}, {:.3f}, {:.3f}) ori=({:.3f}, {:.3f}, {:.3f}, {:.3f})".format(
                STANDOFF_DISTANCE,
                ROBOT_BASE_FRAME,
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z,
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w,
            )
        )
        self.get_logger().info(
            "  Euler (xyz, rad): roll={:.4f}, pitch={:.4f}, yaw={:.4f}".format(
                roll, pitch, yaw
            )
        )
        return target_pose

    def build_goal_constraints(self, target_pose: PoseStamped) -> Constraints:
        """Build position + orientation constraints for the target pose."""
        constraints = Constraints()

        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = END_EFFECTOR_LINK
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [POSITION_TOLERANCE]

        pos_constraint.constraint_region.primitives.append(sphere)
        pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)

        ori_constraint = OrientationConstraint()
        ori_constraint.header = target_pose.header
        ori_constraint.link_name = END_EFFECTOR_LINK
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE
        ori_constraint.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE
        ori_constraint.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE
        ori_constraint.weight = 1.0
        constraints.orientation_constraints.append(ori_constraint)

        return constraints

    def send_move_goal(self, target_pose: PoseStamped):
        """Build and send a MoveGroup goal, then wait for it to finish."""
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(
                "MoveGroup action server (/move_action) not available. "
                "Is move_group running?"
            )

        request = MotionPlanRequest()
        request.workspace_parameters = WorkspaceParameters()
        request.workspace_parameters.header.frame_id = ROBOT_BASE_FRAME
        request.workspace_parameters.min_corner.x = -1.0
        request.workspace_parameters.min_corner.y = -1.0
        request.workspace_parameters.min_corner.z = -1.0
        request.workspace_parameters.max_corner.x = 1.0
        request.workspace_parameters.max_corner.y = 1.0
        request.workspace_parameters.max_corner.z = 1.0

        request.group_name = PLANNING_GROUP
        request.num_planning_attempts = 10
        request.allowed_planning_time = PLANNING_TIME
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1
        request.goal_constraints.append(self.build_goal_constraints(target_pose))

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False  # plan AND execute

        self.get_logger().info("Sending MoveGroup goal...")
        send_goal_future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            raise RuntimeError("MoveGroup goal was rejected.")

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        error_code = result.error_code.val
        if error_code == 1:  # moveit_msgs/MoveItErrorCodes.SUCCESS
            self.get_logger().info(
                "Move succeeded! End-effector is now perpendicular to the marker."
            )
        else:
            raise RuntimeError(f"MoveGroup failed with error code {error_code}.")


def main():
    dry_run = "--dry-run" in sys.argv

    try:
        T_base_camera = load_calibration_transform(CALIBRATION_FILE)
    except RuntimeError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = MoveToArucoNode(T_base_camera)
    try:
        marker_pose_camera = node.wait_for_marker_pose()
        target_pose = node.compute_target_pose(marker_pose_camera)

        if dry_run:
            node.get_logger().info("--dry-run: skipping MoveGroup goal.")
        else:
            node.send_move_goal(target_pose)
    except RuntimeError as e:
        node.get_logger().error(str(e))
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
