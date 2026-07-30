#!/usr/bin/env python3
import os, sys
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, WorkspaceParameters
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped

CALIBRATION_FILE = os.path.expanduser("~/.ros2/easy_handeye2/calibrations/zed_right_arm_calib.calib")
ROBOT_BASE_FRAME = "right_base"
ARUCO_POSE_TOPIC = "/aruco_single/pose"
PLANNING_GROUP = "right_arm"
END_EFFECTOR_LINK = "right_tool0"
PLANNING_TIME = 5.0
POSITION_TOLERANCE = 0.005
ORIENTATION_TOLERANCE = 0.01
POSE_WAIT_TIMEOUT_SEC = 10.0

def load_calibration_transform(path):
    with open(path) as f: data = yaml.safe_load(f)
    t, r = data["transform"]["translation"], data["transform"]["rotation"]
    T = np.eye(4)
    T[:3,:3] = Rotation.from_quat([r["x"],r["y"],r["z"],r["w"]]).as_matrix()
    T[:3,3] = [t["x"],t["y"],t["z"]]
    return T

def pose_to_matrix(pose):
    p, o = pose.pose.position, pose.pose.orientation
    T = np.eye(4)
    T[:3,:3] = Rotation.from_quat([o.x,o.y,o.z,o.w]).as_matrix()
    T[:3,3] = [p.x,p.y,p.z]
    return T

def matrix_to_pose(T, frame_id, stamp):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = T[:3,3]
    qx,qy,qz,qw = Rotation.from_matrix(T[:3,:3]).as_quat()
    pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w = qx,qy,qz,qw
    return pose

class MoveToArucoNode(Node):
    def __init__(self, T_base_camera):
        super().__init__("move_to_aruco")
        self.T_base_camera = T_base_camera
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self.latest_marker_pose = None
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(PoseStamped, ARUCO_POSE_TOPIC, lambda msg: setattr(self, 'latest_marker_pose', msg), qos)

    def wait_for_marker_pose(self):
        self.get_logger().info(f"Waiting for a message on {ARUCO_POSE_TOPIC} ...")
        deadline = self.get_clock().now() + Duration(seconds=POSE_WAIT_TIMEOUT_SEC)
        while rclpy.ok() and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_marker_pose is not None: break
        else: raise RuntimeError(f"Timed out waiting for {ARUCO_POSE_TOPIC}.")
        msg = self.latest_marker_pose
        self.get_logger().info("Marker pose in {}: pos=({:.3f}, {:.3f}, {:.3f})".format(msg.header.frame_id, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
        return msg

    def transform_to_robot_base(self, marker_pose):
        T_base_marker = self.T_base_camera @ pose_to_matrix(marker_pose)
        target_pose = matrix_to_pose(T_base_marker, ROBOT_BASE_FRAME, self.get_clock().now().to_msg())
        q = [target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w]
        roll, pitch, yaw = Rotation.from_quat(q).as_euler("xzy", degrees=False)
        self.get_logger().info("Marker pose in {}: pos=({:.3f}, {:.3f}, {:.3f}) ori=({:.3f}, {:.3f}, {:.3f}, {:.3f})".format(ROBOT_BASE_FRAME, target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z, target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w))
        self.get_logger().info("  Euler (xzy, rad): roll={:.4f}, pitch={:.4f}, yaw={:.4f}".format(roll, pitch, yaw))
        return target_pose

    def build_goal_constraints(self, target_pose):
        constraints = Constraints()
        pc = PositionConstraint()
        pc.header = target_pose.header
        pc.link_name = END_EFFECTOR_LINK
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [POSITION_TOLERANCE]
        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(target_pose.pose)
        pc.weight = 1.0
        constraints.position_constraints.append(pc)
        oc = OrientationConstraint()
        oc.header = target_pose.header
        oc.link_name = END_EFFECTOR_LINK
        oc.orientation = target_pose.pose.orientation
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)
        return constraints

    def send_move_goal(self, target_pose):
        if not self.move_group_client.wait_for_server(timeout_sec=10.0): raise RuntimeError("MoveGroup action server not available.")
        request = MotionPlanRequest()
        request.workspace_parameters = WorkspaceParameters()
        request.workspace_parameters.header.frame_id = ROBOT_BASE_FRAME
        for attr, val in [("min_corner", (-1.0,-1.0,-1.0)), ("max_corner", (1.0,1.0,1.0))]:
            corner = getattr(request.workspace_parameters, attr)
            corner.x, corner.y, corner.z = val
        request.group_name = PLANNING_GROUP
        request.num_planning_attempts = 10
        request.allowed_planning_time = PLANNING_TIME
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1
        request.goal_constraints.append(self.build_goal_constraints(target_pose))
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False
        self.get_logger().info("Sending MoveGroup goal...")
        f = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)
        gh = f.result()
        if not gh.accepted: raise RuntimeError("MoveGroup goal was rejected.")
        self.get_logger().info("Goal accepted, waiting for result...")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        ec = rf.result().result.error_code.val
        if ec == 1: self.get_logger().info("Move succeeded! Arm is now at the marker pose.")
        else: raise RuntimeError(f"MoveGroup failed with error code {ec}.")

def main():
    try: T_base_camera = load_calibration_transform(CALIBRATION_FILE)
    except Exception as e: print(f"ERROR: {e}", file=sys.stderr); sys.exit(1)
    rclpy.init()
    node = MoveToArucoNode(T_base_camera)
    try:
        marker_pose_camera = node.wait_for_marker_pose()
        marker_pose_base = node.transform_to_robot_base(marker_pose_camera)
        node.send_move_goal(marker_pose_base)
    except RuntimeError as e: node.get_logger().error(str(e)); sys.exit(1)
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
