#!/usr/bin/env python3
"""
Collision-aware move helper for left_arm.

When the user drags the RViz interactive marker into a collision zone and
releases the mouse, this node automatically moves the arm as far as possible
along the interpolated path without causing a collision.

If the dragged goal is collision-free the node stays silent — normal
Plan → Execute via the RViz Motion Planning panel works unchanged.
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from moveit_msgs.srv import GetStateValidity, GetPositionIK
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from visualization_msgs.msg import InteractiveMarkerFeedback
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration

import numpy as np


JOINT_NAMES = [
    'left_shoulder_pan_joint',
    'left_shoulder_lift_joint',
    'left_elbow_joint',
    'left_wrist_1_joint',
    'left_wrist_2_joint',
    'left_wrist_3_joint',
]

INTERPOLATION_STEPS = 20
SAFETY_MARGIN = 8


class SafeTrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('safe_trajectory_executor')

        self.cb = ReentrantCallbackGroup()
        self.joint_states = None
        self.is_executing = False

        self.create_subscription(
            JointState, '/joint_states',
            self.joint_state_callback, 10, callback_group=self.cb,
        )
        self.create_subscription(
            InteractiveMarkerFeedback,
            '/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback',
            self.marker_callback, 10, callback_group=self.cb,
        )

        self.validity_client = self.create_client(
            GetStateValidity, '/check_state_validity', callback_group=self.cb,
        )
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=self.cb,
        )
        self.controller_client = ActionClient(
            self, FollowJointTrajectory,
            '/left_arm_controller/follow_joint_trajectory',
            callback_group=self.cb,
        )

        self.get_logger().info('=' * 50)
        self.get_logger().info('Safe Trajectory Executor ready.')
        self.get_logger().info(f'Steps: {INTERPOLATION_STEPS}, margin: {SAFETY_MARGIN}')
        self.get_logger().info('Drag into collision zone + release → auto-move to last safe point.')
        self.get_logger().info('Drag to valid position → use Plan + Execute as normal.')
        self.get_logger().info('=' * 50)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def joint_state_callback(self, msg: JointState):
        self.joint_states = msg

    def marker_callback(self, msg: InteractiveMarkerFeedback):
        # MoveIt2 names the left_arm goal marker "EE:goal_left_tcp" or "EEF:left_arm:left_tcp"
        marker = msg.marker_name
        if not any(s in marker for s in ('left_arm', 'left_tcp', 'left_tool0')):
            return

        if msg.event_type != 5:  # MOUSE_UP only
            return

        if self.is_executing:
            self.get_logger().warn('Already executing — ignoring marker release.')
            return

        pos = msg.pose.position
        ori = msg.pose.orientation
        frame = msg.header.frame_id or 'world'

        threading.Thread(
            target=self.run_case_b,
            args=([pos.x, pos.y, pos.z], [ori.x, ori.y, ori.z, ori.w], frame),
            daemon=True,
        ).start()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def get_current_joints(self):
        if self.joint_states is None:
            return None
        pos = dict(zip(self.joint_states.name, self.joint_states.position))
        try:
            return np.array([pos[j] for j in JOINT_NAMES])
        except KeyError:
            return None

    def _wait_future(self, future, timeout_sec=3.0) -> bool:
        """Poll a ROS2 future without re-spinning the already-running executor."""
        deadline = time.monotonic() + timeout_sec
        while not future.done():
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.005)
        return True

    def check_waypoint(self, positions) -> bool:
        """Return True if the joint configuration is collision-free."""
        if not self.validity_client.wait_for_service(timeout_sec=1.0):
            return False
        req = GetStateValidity.Request()
        req.group_name = 'left_arm'
        req.robot_state.joint_state.name = list(JOINT_NAMES)
        req.robot_state.joint_state.position = list(positions)
        req.robot_state.is_diff = True
        future = self.validity_client.call_async(req)
        if not self._wait_future(future):
            return False
        result = future.result()
        return result.valid if result is not None else False

    def build_trajectory(self, points_list, time_step_sec=0.2):
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        for i, positions in enumerate(points_list):
            pt = JointTrajectoryPoint()
            pt.positions = list(positions)
            pt.velocities = [0.0] * 6
            pt.accelerations = [0.0] * 6
            ns = int(i * time_step_sec * 1e9)
            pt.time_from_start = Duration(sec=ns // 10**9, nanosec=ns % 10**9)
            traj.points.append(pt)
        return traj

    def send_trajectory(self, traj: JointTrajectory) -> bool:
        if not self.controller_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Controller not available!')
            return False

        send_future = self.controller_client.send_goal_async(
            FollowJointTrajectory.Goal(trajectory=traj)
        )
        if not self._wait_future(send_future, timeout_sec=5.0):
            return False

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Controller rejected trajectory!')
            return False

        self.get_logger().info('Trajectory accepted. Executing...')
        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, timeout_sec=60.0):
            self.get_logger().error('Controller timed out!')
            return False

        self.get_logger().info('Controller finished.')
        return True

    # ------------------------------------------------------------------
    # Case B — collision-zone drag
    # ------------------------------------------------------------------

    def run_case_b(self, goal_position, goal_orientation, frame='world'):
        if self.is_executing:
            return
        self.is_executing = True

        try:
            self._do_case_b(goal_position, goal_orientation, frame)
        finally:
            self.is_executing = False

    def _do_case_b(self, goal_position, goal_orientation, frame):
        current_joints = self.get_current_joints()
        if current_joints is None:
            self.get_logger().error('No joint states available.')
            return

        if not self.ik_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('IK service not available!')
            return

        req = GetPositionIK.Request()
        req.ik_request.group_name = 'left_arm'
        req.ik_request.avoid_collisions = False
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = goal_position
        pose.pose.orientation.x, pose.pose.orientation.y = goal_orientation[0], goal_orientation[1]
        pose.pose.orientation.z, pose.pose.orientation.w = goal_orientation[2], goal_orientation[3]
        req.ik_request.pose_stamped = pose
        req.ik_request.robot_state.joint_state.name = JOINT_NAMES
        req.ik_request.robot_state.joint_state.position = current_joints.tolist()

        future = self.ik_client.call_async(req)
        if not self._wait_future(future, timeout_sec=5.0):
            self.get_logger().error('IK timed out.')
            return
        if future.result() is None or future.result().error_code.val != 1:
            self.get_logger().error('IK failed — goal may be out of reach.')
            return

        goal_joint_map = dict(zip(
            future.result().solution.joint_state.name,
            future.result().solution.joint_state.position,
        ))
        goal_joints = np.array([goal_joint_map[j] for j in JOINT_NAMES])

        # If the goal itself is collision-free, stay silent — let the user
        # Plan + Execute normally via the RViz motion planning panel.
        if self.check_waypoint(goal_joints):
            self.get_logger().info('Goal is collision-free — use Plan + Execute in RViz.')
            return

        self.get_logger().info(f'Goal is in collision. Checking {INTERPOLATION_STEPS} interpolated steps...')

        last_safe_idx = -1
        for i in range(1, INTERPOLATION_STEPS + 1):
            alpha = i / INTERPOLATION_STEPS
            pt = (1 - alpha) * current_joints + alpha * goal_joints
            if self.check_waypoint(pt):
                last_safe_idx = i
                self.get_logger().info(f'  [{i}/{INTERPOLATION_STEPS}] safe ✓')
            else:
                self.get_logger().warn(f'  [{i}/{INTERPOLATION_STEPS}] COLLISION — stopping search.')
                break

        if last_safe_idx == -1:
            self.get_logger().error('No safe position along path. Not moving.')
            return

        safe_idx = max(0, last_safe_idx - SAFETY_MARGIN)
        alpha = safe_idx / INTERPOLATION_STEPS
        target_joints = (1 - alpha) * current_joints + alpha * goal_joints
        self.get_logger().info(
            f'Moving to step {safe_idx}/{INTERPOLATION_STEPS} '
            f'(safety margin {SAFETY_MARGIN} steps back from collision).'
        )

        n = 10
        points = [(1 - a / n) * current_joints + (a / n) * target_joints for a in range(n + 1)]
        traj = self.build_trajectory(points, time_step_sec=0.2)
        self.get_logger().info('Sending trajectory to controller...')
        success = self.send_trajectory(traj)
        self.get_logger().info(f'Case B done. Success: {success}')


def main():
    rclpy.init()
    node = SafeTrajectoryExecutor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
