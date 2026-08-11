"""MoveIt 2 plan-only client for the ``left_arm`` planning group.

Sends a ``MotionPlanRequest`` to the ``/move_action`` action server with
``plan_only = True``.  The move_group node automatically publishes the
resulting trajectory to ``/move_group/display_planned_path``, which RViz
renders as a ghost trajectory — no motion occurs on the real hardware.

End-effector
------------
``left_tcp`` — a fixed link 0.244 m along ``+Z`` from ``left_tool0``,
defined in the URDF and registered as the ``left_arm`` tip link in the SRDF.
MoveIt solves IK to this frame directly.

``right_tcp`` — a fixed link 0.244 m along ``+Z`` from ``right_tool0``,
defined in the URDF and registered as the ``right_arm`` tip link in the SRDF.
MoveIt solves IK to this frame directly.

Start state
-----------
``req.start_state`` is always populated explicitly with the SRDF home
positions for both arms.  This is required when physical joint states are
unavailable (hardware not connected / ``joint_state_broadcaster`` not running)
because an empty ``start_state`` would leave ``DisplayTrajectory.trajectory_start``
empty, preventing RViz from animating the planned path.
"""

from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    DisplayTrajectory,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
    WorkspaceParameters,
    PlanningScene,
    CollisionObject,
    JointConstraint,
    MoveItErrorCodes,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
import rclpy



# ── Planning constants ────────────────────────────────────────────────────────

_PLANNING_GROUP: str = "left_arm"
_EEF_LINK: str = "left_tcp"
_PLANNING_FRAME: str = "left_base"

_RIGHT_PLANNING_GROUP: str = "right_arm"
_RIGHT_EEF_LINK: str = "right_tcp"
_RIGHT_PLANNING_FRAME: str = "right_base"

_PLANNING_TIME_SEC: float = 10.0
_NUM_ATTEMPTS: int = 10
_MAX_VEL_SCALE: float = 0.1
_MAX_ACCEL_SCALE: float = 0.1

# Tolerance sphere radius for position constraint (metres).
_POSITION_TOL_M: float = 0.01
# Per-axis tolerance for orientation constraint (radians).
_ORIENTATION_TOL_RAD: float = 0.1

# Half-side of the axis-aligned planning workspace cube (metres).
_WORKSPACE_HALF_M: float = 2.0

# Action server name (move_group default).
_MOVE_ACTION: str = "/move_action"
_ACTION_SERVER_TIMEOUT_SEC: float = 30.0

# Topic on which move_group and this node publish trajectory visualizations.
_DISPLAY_TOPIC: str = "/move_group/display_planned_path"

# How often (seconds) to re-publish the planned trajectory so RViz loops the
# animation continuously without requiring any RViz "Loop Animation" config.
_DISPLAY_LOOP_SEC: float = 4.0

# Home joint positions taken from the SRDF left_home / right_home named states.
# Used as the explicit start_state so move_group always has a valid reference
# configuration and RViz receives a populated trajectory_start for animation.
_HOME_JOINTS: dict = {
    "left_shoulder_pan_joint":   -0.5842,
    "left_shoulder_lift_joint":  -1.4352,
    "left_elbow_joint":          -2.4564,
    "left_wrist_1_joint":         0.5894,
    "left_wrist_2_joint":        -4.1043,
    "left_wrist_3_joint":        -1.7400,
    "right_shoulder_pan_joint":  -2.5287,
    "right_shoulder_lift_joint": -1.9195,
    "right_elbow_joint":          2.5167,
    "right_wrist_1_joint":       -3.4194,
    "right_wrist_2_joint":       -2.0268,
    "right_wrist_3_joint":        0.2502,
}


class MoveItPlanOnlyClient(Node):
    """ROS 2 node that requests a plan-only trajectory from move_group.

    After the node is constructed the ``/move_action`` server is already
    connected and ready.  Call :meth:`plan_to_pose` to trigger planning.

    Args:
        node_name: Optional override for the ROS 2 node name.
    """

    def __init__(self, node_name: str = "aruco_moveit_planner") -> None:
        super().__init__(node_name)
        self._client: ActionClient = ActionClient(self, MoveGroup, _MOVE_ACTION)
        self._display_pub = self.create_publisher(DisplayTrajectory, _DISPLAY_TOPIC, 1)
        self._stored_trajectory = None   # populated after a successful plan
        self._display_timer = None       # periodic republish timer
        self._exec_client = ActionClient(self, ExecuteTrajectory, "execute_trajectory")

        self.get_logger().info(
            f"Waiting for '{_MOVE_ACTION}' action server "
            f"(timeout={_ACTION_SERVER_TIMEOUT_SEC:.0f}s)…"
        )
        if not self._client.wait_for_server(timeout_sec=_ACTION_SERVER_TIMEOUT_SEC):
            raise RuntimeError(
                f"'{_MOVE_ACTION}' is not available. "
                "Ensure dual_arm_real_v2.launch.py is running "
                "and move_group has started."
            )
        self.get_logger().info("Connected to move_group action server.")

        self._planning_scene_pub = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )

        self._current_joint_state = None
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )

    # ── Public API ────────────────────────────────────────────────────────────

    def plan_to_pose(self, target_pose: PoseStamped) -> bool:
        """Request a plan to *target_pose* for the ``left_tcp`` end-effector.

        The call is **synchronous** — it blocks until move_group returns a
        result.  The trajectory is published to
        ``/move_group/display_planned_path`` for RViz and is **not** executed
        on the real robot (``plan_only = True``).

        Args:
            target_pose: Goal pose for ``left_tcp`` expressed in ``left_base``.

        Returns:
            ``True`` if planning succeeded, ``False`` otherwise.
        """
        goal = MoveGroup.Goal()
        goal.planning_options.plan_only = False
        goal.request = self._build_request(target_pose)

        p = target_pose.pose.position
        self.get_logger().info(
            f"[plan] Requesting plan-only trajectory for '{_EEF_LINK}' "
            f"→ pos=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) "
            f"in frame '{target_pose.header.frame_id}'"
        )

        import rclpy

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                "[plan] move_group rejected the goal. "
                "Check that the planning group and EEF link exist in the SRDF."
            )
            return False

        self.get_logger().info("[plan] Goal accepted — waiting for plan result…")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code.val == 1:  # SUCCESS
            self._stored_trajectory = result.planned_trajectory
        return self._report_result(result.error_code.val)

    def plan_to_pose2(self, target_pose: PoseStamped,
                  constrain_joints: bool = True,
                  timeout_sec: float = 30.0) -> bool:
        """Plan only. Stores the trajectory in self._stored_trajectory.
        Nothing moves until execute_stored_trajectory() is called."""
        goal = MoveGroup.Goal()
        goal.planning_options.plan_only = True          # <-- the key line
        goal.request = self._build_request(
            target_pose,
            group=_RIGHT_PLANNING_GROUP,
            eef_link=_RIGHT_EEF_LINK,
            planning_frame=_RIGHT_PLANNING_FRAME,
            constrain_joints=constrain_joints,
        )

        p = target_pose.pose.position
        self.get_logger().info(
            f"[plan] Planning for '{_RIGHT_EEF_LINK}' → "
            f"({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) in '{target_pose.header.frame_id}'"
        )

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("[plan] Goal rejected or no response.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if result_future.result() is None:
            self.get_logger().error("[plan] No result (timeout?).")
            return False

        result = result_future.result().result
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self._stored_trajectory = result.planned_trajectory
            self.get_logger().info(
                "[plan] Success — trajectory stored. Check RViz, then call "
                "execute_stored_trajectory() to run it."
            )
            return True

        self.get_logger().error(f"[plan] Failed, error code {result.error_code.val}")
        return False


    def execute_stored_trajectory(self, timeout_sec: float = 60.0) -> bool:
        """Execute the trajectory from the last successful plan."""
        if  self._stored_trajectory is None:
            self.get_logger().error("[exec] No stored trajectory — plan first.")
            return False

        if  not self._exec_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("[exec] /execute_trajectory server not available.")
            return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = self._stored_trajectory

        self.get_logger().info("[exec] Executing stored trajectory…")
        send_future = self._exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)
        goal_handle = send_future.result()
        if  goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("[exec] Execution goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if  result_future.result() is None:
            self.get_logger().error("[exec] No execution result (timeout?).")
            return False

        ok = result_future.result().result.error_code.val == MoveItErrorCodes.SUCCESS
        if ok:
            self.get_logger().info("[exec] Motion complete.")
            # self._stored_trajectory = None   # consume it — prevents double-execution
        else:
            self.get_logger().error("[exec] Execution failed.")
        return ok

    def start_display_loop(self) -> None:
        """Repeatedly publish the last planned trajectory to ``/move_group/display_planned_path``.

        RViz plays the trajectory animation once when a ``DisplayTrajectory``
        message arrives, then returns to showing the current robot state.  By
        re-publishing every :data:`_DISPLAY_LOOP_SEC` seconds the animation
        keeps running continuously — no RViz "Loop Animation" checkbox needed.

        Call once after :meth:`plan_to_pose` returns ``True``, then
        ``rclpy.spin`` the node until the user presses Ctrl+C.
        """
        if self._stored_trajectory is None:
            self.get_logger().warn("[display] No trajectory stored — call plan_to_pose first.")
            return
        self._publish_display()  # immediate first publish
        self._display_timer = self.create_timer(_DISPLAY_LOOP_SEC, self._publish_display)
        self.get_logger().info(
            f"[display] Looping trajectory in RViz every {_DISPLAY_LOOP_SEC:.0f}s. "
            "Press Ctrl+C to stop."
        )

    # ── Private helpers ───────────────────────────────────────────────────────
    def _joint_state_cb(self, msg: JointState) -> None:
        self._current_joint_state = msg

    def _publish_display(self) -> None:
        """Emit one ``DisplayTrajectory`` message with the stored planned path."""
        if self._stored_trajectory is None:
            return
        msg = DisplayTrajectory()
        msg.trajectory_start = self._build_home_start_state()
        msg.trajectory = [self._stored_trajectory]
        self._display_pub.publish(msg)

    def _build_request(self, target_pose, group=_PLANNING_GROUP, eef_link=_EEF_LINK, planning_frame=_PLANNING_FRAME,
    pipeline_id="ompl", planner_id="RRTstarkConfigDefault", constrain_joints: bool = True) -> MotionPlanRequest:
        """Assemble a complete ``MotionPlanRequest`` for *target_pose*.

        Args:
            target_pose: Goal pose for ``left_tcp``.

        Returns:
            Fully populated ``MotionPlanRequest``.
        """
        req = MotionPlanRequest()
        req.group_name = group
        req.planner_id = planner_id
        req.pipeline_id = pipeline_id
        req.num_planning_attempts = _NUM_ATTEMPTS
        req.allowed_planning_time = _PLANNING_TIME_SEC
        req.max_velocity_scaling_factor = _MAX_VEL_SCALE
        req.max_acceleration_scaling_factor = _MAX_ACCEL_SCALE
        req.workspace_parameters = self._build_workspace(planning_frame)
        req.goal_constraints.append(self._build_goal_constraints(target_pose, eef_link))

        if self._current_joint_state is None:
            import rclpy
            self.get_logger().info("[plan] Waiting for /joint_states…")
            deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=3.0)
            while self._current_joint_state is None and self.get_clock().now() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            if self._current_joint_state is None:
                self.get_logger().warn("[plan] No joint states received — joint constraints skipped.")

        if constrain_joints and self._current_joint_state is not None and group == _RIGHT_PLANNING_GROUP:
            watch_joints = {
                "right_shoulder_pan_joint": 1.7854,
                "right_wrist_1_joint":      1.0000,
                # "right_wrist_2_joint":      0.7854,
                "right_wrist_3_joint":      1.7854,
            }
            pc = Constraints()
            for jname, tol in watch_joints.items():
                if jname not in self._current_joint_state.name:
                    continue
                idx = self._current_joint_state.name.index(jname)
                angle = self._current_joint_state.position[idx]
                jc = JointConstraint()
                jc.joint_name = jname
                jc.position = angle
                jc.tolerance_above = tol
                jc.tolerance_below = tol
                jc.weight = 1.0
                req.goal_constraints[0].joint_constraints.append(jc)
                pc.joint_constraints.append(jc)
            req.path_constraints = pc
        return req

    def _build_home_start_state(self) -> RobotState:
        """Return a ``RobotState`` with both arms at their SRDF home positions.

        Populating ``start_state`` explicitly ensures that:

        * move_group plans from a well-defined configuration, not an empty one.
        * ``DisplayTrajectory.trajectory_start`` is populated so RViz can
          animate the ghost trajectory from the home pose to the goal pose.

        Returns:
            ``RobotState`` with all arm joints set to their home angles.
        """
        js = JointState()
        js.name = list(_HOME_JOINTS.keys())
        js.position = list(_HOME_JOINTS.values())
        state = RobotState()
        state.joint_state = js
        return state

    def _build_workspace(self, frame=_PLANNING_FRAME):
        """Return a cubic workspace envelope centred at the origin."""
        ws = WorkspaceParameters()
        ws.header.frame_id = frame
        ws.min_corner.x = ws.min_corner.y = ws.min_corner.z = -_WORKSPACE_HALF_M
        ws.max_corner.x = ws.max_corner.y = ws.max_corner.z = _WORKSPACE_HALF_M
        return ws

    def _build_goal_constraints(self, target_pose: PoseStamped, eef_link=_EEF_LINK) -> Constraints:
        """Build paired position + orientation constraints for *target_pose*.

        Args:
            target_pose: The desired ``left_tcp, right_tcp`` pose.

        Returns:
            ``Constraints`` message with one position and one orientation entry.
        """
        constraints = Constraints()

        # Position constraint — tolerance sphere centred at the marker position.
        sphere = SolidPrimitive(
            type=SolidPrimitive.SPHERE,
            dimensions=[_POSITION_TOL_M],
        )
        pos_c = PositionConstraint()
        pos_c.header = target_pose.header
        pos_c.link_name = eef_link
        pos_c.constraint_region = BoundingVolume(
            primitives=[sphere],
            primitive_poses=[target_pose.pose],
        )
        pos_c.weight = 1.0
        constraints.position_constraints.append(pos_c)

        # Orientation constraint — tight per-axis tolerances.
        ori_c = OrientationConstraint()
        ori_c.header = target_pose.header
        ori_c.link_name = eef_link
        ori_c.orientation = target_pose.pose.orientation
        ori_c.absolute_x_axis_tolerance = _ORIENTATION_TOL_RAD
        ori_c.absolute_y_axis_tolerance = _ORIENTATION_TOL_RAD
        ori_c.absolute_z_axis_tolerance = _ORIENTATION_TOL_RAD
        ori_c.weight = 1.0
        constraints.orientation_constraints.append(ori_c)

        return constraints

    def _report_result(self, error_code: int) -> bool:
        """Log a human-readable outcome and return success flag.

        Args:
            error_code: ``MoveItErrorCodes.val`` from the action result.

        Returns:
            ``True`` on success (code == 1), ``False`` otherwise.
        """
        if error_code == 1:  # MoveItErrorCodes.SUCCESS
            self.get_logger().info(
                "[plan] SUCCESS — trajectory published to "
                "/move_group/display_planned_path. "
                "Open RViz → MotionPlanning panel to inspect."
            )
            return True

        # Provide human-readable error labels for the most common codes.
        _LABELS = {
            -1: "FAILURE",
            -2: "PLANNING_FAILED",
            -3: "INVALID_MOTION_PLAN",
            -5: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
            -10: "CONTROL_FAILED",
            -12: "UNABLE_TO_AQUIRE_SENSOR_DATA",
            -13: "TIMED_OUT",
            -14: "PREEMPTED",
            99999: "NO_IK_SOLUTION",
        }
        label = _LABELS.get(error_code, f"code={error_code}")
        self.get_logger().error(
            f"[plan] FAILED ({label}). "
            "Check move_group logs for details. "
            "Possible causes: IK unreachable, joint limits, collision."
        )
        return False

    def _add_cylinder_obstacle(self, name, x, y, z, height, radius, frame="world"):
        """Add a cylindrical collision object to the planning scene.

        Args:
            name: Unique name for the object.
            x, y, z: Cylinder centre position in *frame*.
            height: Cylinder height along its local Z axis.
            radius: Cylinder radius.
            frame: Reference frame for the cylinder pose.
        """
        obj = CollisionObject()
        obj.header.frame_id = frame
        obj.id = name

        cylinder = SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[height, radius])

        cylinder_pose = Pose()
        cylinder_pose.position.x = x
        cylinder_pose.position.y = y
        cylinder_pose.position.z = z
        cylinder_pose.orientation.w = 1.0  # no rotation

        obj.primitives.append(cylinder)
        obj.primitive_poses.append(cylinder_pose)
        obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.world.collision_objects.append(obj)
        scene.is_diff = True  # only update the diff, not the entire scene
        self._planning_scene_pub.publish(scene)

        self.get_logger().info(
            f"[scene] Added cylinder '{name}' at ({x:.3f}, {y:.3f}, {z:.3f}) "
            f"height={height:.3f} radius={radius:.3f} in frame '{frame}'."
        )

    def _add_box_obstacle(self, name, x, y, z, length, width, height, frame="world"):
        """Add a box collision object to the planning scene.

        Args:
            name: Unique name for the object.
            x, y, z: Box centre position in *frame*.
            length: Box dimension along local X axis.
            width:  Box dimension along local Y axis.
            height: Box dimension along local Z axis.
            frame: Reference frame for the box pose.
        """
        obj = CollisionObject()
        obj.header.frame_id = frame
        obj.id = name

        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[length, width, height])

        box_pose = Pose()
        box_pose.position.x = x
        box_pose.position.y = y
        box_pose.position.z = z
        box_pose.orientation.w = 1.0  # no rotation

        obj.primitives.append(box)
        obj.primitive_poses.append(box_pose)
        obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.world.collision_objects.append(obj)
        scene.is_diff = True  # only update the diff, not the entire scene

        self._planning_scene_pub.publish(scene)

        self.get_logger().info(
            f"[scene] Added box '{name}' at ({x:.3f}, {y:.3f}, {z:.3f}) "
            f"size=({length:.3f}, {width:.3f}, {height:.3f}) in frame '{frame}'."
        )

    def _remove_obstacle(self, name, frame="world"):
        """Remove a collision object from the planning scene.

        Args:
            name: Name of the object to remove.
            frame: Reference frame for the object.
        """
        obj = CollisionObject()
        obj.header.frame_id = frame
        obj.id = name
        obj.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.world.collision_objects.append(obj)
        scene.is_diff = True  # only update the diff, not the entire scene
        self._planning_scene_pub.publish(scene)

        self.get_logger().info(f"[scene] Removed collision object '{name}' from frame '{frame}'.")