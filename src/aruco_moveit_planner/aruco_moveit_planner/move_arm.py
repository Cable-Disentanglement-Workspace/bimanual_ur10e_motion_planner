"""Move arm node: plan then execute (with confirmation) a Cartesian goal or delta.

Motion planning algorithms
--------------------------
Two planners are available, selected per call:

**Pilz LIN** (default for delta and absolute Cartesian moves)
  ``pipeline_id = "pilz_industrial_motion_planner"``  ``planner_id = "LIN"``
  Interpolates the end-effector path as a **straight line** in Cartesian space.
  Orientation is also interpolated linearly (SLERP) from start to goal.
  Use this for any move where you want the tool to travel in a straight line —
  e.g. +10 cm in Z.  Velocity/acceleration limits come from
  ``pilz_cartesian_limits.yaml``.

**OMPL RRTConnect** (used for ``--home`` and ``--joint-space`` moves)
  ``pipeline_id = "ompl"``  ``planner_id = "RRTConnectkConfigDefault"``
  Plans in **joint space** — fast and flexible, finds paths through complex
  configurations.  The EEF Cartesian path is NOT guaranteed to be a straight
  line; for a simple +Z delta it may rotate or arc slightly.
  Use this when LIN fails (singularity, collision along the straight line)
  or when returning to a named joint configuration (home).

Pass ``--joint-space`` on the CLI to force OMPL for any move.

Execution flow
--------------
1. Planning  — MoveIt plans a collision-free joint trajectory.
2. Preview   — ghost trajectory displayed in RViz MoveIt panel,
               EEF path shown as a cyan LINE_STRIP marker.
3. Confirm   — press Enter to execute, Ctrl+C to abort.
4. Execute   — the exact planned trajectory is sent to the controller.

Where to input x y z rx ry rz
---------------------------------
**In the code** — edit the block labelled ``HARDCODED TARGET`` in ``main()``
below. Set ``_X, _Y, _Z`` (metres) and ``_ROLL, _PITCH, _YAW`` (radians).

**On the command line** — pass ``--target X Y Z ROLL PITCH YAW``:
    ros2 run aruco_moveit_planner move_arm \\
        --target 0.3 0.1 0.8 0.0 1.57 0.0

**Via launch file** — named arguments:
    ros2 launch aruco_moveit_planner move_arm.launch.py \\
        x:=0.3 y:=0.1 z:=0.8 pitch:=1.57

All coordinates are in the arm's **base frame**
(``right_base`` for the right arm, ``left_base`` for the left arm).
Orientation is intrinsic ZYX roll-pitch-yaw in radians.
"""

import argparse
import math
import sys

import rclpy
import rclpy.duration
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    DisplayTrajectory,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
    WorkspaceParameters,
)
from moveit_msgs.srv import GetPositionFK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

# ── Arm configurations ─────────────────────────────────────────────────────────

_ARM_CONFIG = {
    "right": {
        "group":       "right_arm",
        "eef_link":    "right_tool0",
        "base_frame":  "right_base",
        "joint_names": [
            "right_shoulder_pan_joint",
            "right_shoulder_lift_joint",
            "right_elbow_joint",
            "right_wrist_1_joint",
            "right_wrist_2_joint",
            "right_wrist_3_joint",
        ],
    },
    "left": {
        "group":       "left_arm",
        "eef_link":    "left_tcp",
        "base_frame":  "left_base",
        "joint_names": [
            "left_shoulder_pan_joint",
            "left_shoulder_lift_joint",
            "left_elbow_joint",
            "left_wrist_1_joint",
            "left_wrist_2_joint",
            "left_wrist_3_joint",
        ],
    },
}

# Home joint positions from the SRDF (both arms)
_HOME_JOINTS: dict = {
    "left_shoulder_pan_joint":   -0.9579,
    "left_shoulder_lift_joint":  -1.9195,
    "left_elbow_joint":           2.5167,
    "left_wrist_1_joint":        -3.4194,
    "left_wrist_2_joint":        -2.0268,
    "left_wrist_3_joint":         0.2502,
    "right_shoulder_pan_joint":  -2.155,
    "right_shoulder_lift_joint": -1.4352,
    "right_elbow_joint":         -2.4564,
    "right_wrist_1_joint":        0.5894,
    "right_wrist_2_joint":       -4.1043,
    "right_wrist_3_joint":       -0.2312,
}

# ── Planning constants ─────────────────────────────────────────────────────────

_PLANNING_TIME_SEC:       float = 10.0
_PLANNING_TIME_OMPL_OPT:  float = 30.0   # RRTstar runs the full window to optimise
_NUM_ATTEMPTS:            int   = 50   # higher count helps IK sampling find the constrained range
_MAX_VEL_SCALE:       float = 0.1
_MAX_ACCEL_SCALE:     float = 0.1
_POSITION_TOL_M:      float = 0.001
_ORIENTATION_TOL_RAD: float = 0.01
_JOINT_TOL_RAD:       float = 0.01
_WORKSPACE_HALF_M:    float = 2.0
_MOVE_ACTION:         str   = "/move_action"
_EXECUTE_ACTION:      str   = "/execute_trajectory"
_FK_SERVICE:          str   = "/compute_fk"
_ACTION_TIMEOUT_SEC:  float = 30.0
_TF_WAIT_SEC:         float = 10.0
_JS_WAIT_SEC:         float = 5.0
_FK_TIMEOUT_SEC:      float = 2.0

_MARKER_MAX_POINTS:   int   = 25
_MARKER_LIFETIME_SEC: float = 60.0
_MARKER_TOPIC:        str   = "/move_arm/eef_path"

# ── Per-joint path constraints ─────────────────────────────────────────────────
# Applied during OMPL planning (ompl / ompl-opt) to limit joint travel.
# Has no effect with Pilz LIN / PTP.
#
# Each entry: joint_name → (min_rad, max_rad)  — absolute joint angle bounds.
# The OMPL planner will only find paths where every waypoint stays inside these
# ranges.  If the robot's current joint value is outside a range, planning
# will immediately fail — so set ranges that comfortably include the start pose.
#
# Tip: use the slider GUI (starts automatically with dual_arm_fake.launch.py)
# to move the robot to poses you care about and read off the joint angles.
# Those angles become your min/max values here.
#
# Set _USE_JOINT_PATH_LIMITS = True to enable.

_USE_JOINT_PATH_LIMITS: bool = False  # see note below — use --planner ptp instead

_JOINT_PATH_LIMITS: dict = {
    # ── right arm ────────────────────────────────────── (min_rad,  max_rad)
    "right_shoulder_pan_joint":  (-3.14,  +0.785 ),   # -180° … 0°
    "right_shoulder_lift_joint": (-2.70,  +0.5 ),   # -155° … -29°
    "right_elbow_joint":         (-3.14,  +0.5 ),   # -180° … 0°
    "right_wrist_1_joint":       (-1.57,  +1.57),   # +17°  … +52°
    "right_wrist_2_joint":       (-4.60,  -3.14),   # -252° … -218°
    "right_wrist_3_joint":       (-3.80,  +3.40),   # -29°  … +6°
    # ── left arm ─────────────────────────────────────── (min_rad,  max_rad)
    "left_shoulder_pan_joint":   (-2.00,   0.0 ),
    "left_shoulder_lift_joint":  (-2.80,  -1.00),
    "left_elbow_joint":          (+1.50,  +3.14),
    "left_wrist_1_joint":        (-4.00,  -2.50),
    "left_wrist_2_joint":        (-2.60,  -1.50),
    "left_wrist_3_joint":        (-0.50,  +1.00),
}

_MOVEIT_ERROR_LABELS: dict = {
    -1:    "FAILURE",
    -2:    "PLANNING_FAILED",
    -3:    "INVALID_MOTION_PLAN",
    -5:    "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -10:   "CONTROL_FAILED",
    -12:   "UNABLE_TO_ACQUIRE_SENSOR_DATA",
    -13:   "TIMED_OUT",
    -14:   "PREEMPTED",
    99999: "NO_IK_SOLUTION",
}

# ── Math helpers ───────────────────────────────────────────────────────────────


def _rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    q = Quaternion()
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    q.w = cr * cp * cy + sr * sp * sy
    return q


def _quat_to_rpy(q) -> tuple:
    """Return (roll, pitch, yaw) in radians from a geometry_msgs Quaternion."""
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)

    return roll, pitch, yaw


def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = r, g, b, a
    return c


# ── Main node ─────────────────────────────────────────────────────────────────


class MoveArmClient(Node):
    """ROS 2 node that plans then (after confirmation) executes a Cartesian goal.

    Always plans first and shows the trajectory in RViz.
    Prompts the user to press Enter before sending to the controller.
    Use ``plan_only=True`` to skip the prompt and never execute.

    Args:
        arm:     ``"right"`` or ``"left"``
        offline: Use home joints as start state (no real robot needed).
    """

    def __init__(self, arm: str = "right", offline: bool = False) -> None:
        super().__init__("move_arm_client")

        cfg = _ARM_CONFIG[arm]
        self._group:          str  = cfg["group"]
        self._eef_link:       str  = cfg["eef_link"]
        self._base_frame:     str  = cfg["base_frame"]
        self._arm_joint_names: list = cfg["joint_names"]
        self._offline:        bool = offline

        self._latest_joints: dict | None = None

        self._client         = ActionClient(self, MoveGroup,          _MOVE_ACTION)
        self._execute_client = ActionClient(self, ExecuteTrajectory,   _EXECUTE_ACTION)
        self._display_pub    = self.create_publisher(
            DisplayTrajectory, "/move_group/display_planned_path", 1
        )
        self._marker_pub     = self.create_publisher(Marker, _MARKER_TOPIC, 10)
        self._fk_client      = self.create_client(GetPositionFK, _FK_SERVICE)

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        if not offline:
            self._js_sub = self.create_subscription(
                JointState, "/joint_states", self._on_joint_state, 10
            )

        self.get_logger().info(
            f"[{self._group}] mode={'offline' if offline else 'online'} | "
            f"eef={self._eef_link} | base={self._base_frame}"
        )
        self.get_logger().info(
            f"Waiting for '{_MOVE_ACTION}' (timeout={_ACTION_TIMEOUT_SEC:.0f}s)…"
        )
        if not self._client.wait_for_server(timeout_sec=_ACTION_TIMEOUT_SEC):
            raise RuntimeError(
                f"'{_MOVE_ACTION}' not available. "
                "Ensure the launch file is running and move_group has started."
            )
        self.get_logger().info("Connected to move_group.")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_joint_state(self, msg: JointState) -> None:
        joints = dict(zip(msg.name, msg.position))
        if all(j in joints for j in self._arm_joint_names):
            self._latest_joints = joints

    # ── Public API ────────────────────────────────────────────────────────────

    def move_to_home(self, plan_only: bool = False) -> bool:
        """Plan (then confirm + execute) a return to the SRDF home configuration."""
        if not self._offline:
            self._wait_for_joint_states()

        # Current EEF from TF
        current = self._get_eef_pose_from_tf()
        self._print_pose("Current EEF", current.pose)
        src = self._latest_joints or _HOME_JOINTS
        self._print_joints(
            "Current joints",
            self._arm_joint_names,
            [src.get(n, _HOME_JOINTS[n]) for n in self._arm_joint_names],
        )

        # Home EEF from FK of the SRDF home joint values
        home_fk = self._call_fk(
            self._arm_joint_names,
            [_HOME_JOINTS[n] for n in self._arm_joint_names],
        )
        if home_fk is not None:
            self._print_pose("Target  EEF (home)", home_fk.pose)
        else:
            print("\n  Target EEF (home): FK unavailable — moving by joint constraints only.")
        self._print_joints(
            "Target  joints (home)",
            self._arm_joint_names,
            [_HOME_JOINTS[n] for n in self._arm_joint_names],
        )

        constraints = Constraints()
        for name in self._arm_joint_names:
            jc = JointConstraint()
            jc.joint_name      = name
            jc.position        = _HOME_JOINTS[name]
            jc.tolerance_above = _JOINT_TOL_RAD
            jc.tolerance_below = _JOINT_TOL_RAD
            jc.weight          = 1.0
            constraints.joint_constraints.append(jc)

        return self._run(self._build_base_request(), constraints, plan_only)

    def move_delta(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.10,
        plan_only: bool = False,
        planner: str = "lin",
    ) -> bool:
        """Move EEF by (dx, dy, dz) metres in the arm base frame, keeping orientation.

        planner: "ptp" (default) — shortest joint-space path, minimises rotation.
                 "lin"           — straight Cartesian line.
                 "ompl"          — OMPL RRTConnect fallback.
                 "ompl-opt"      — OMPL RRTstar, optimises for minimal path length.
        """
        if not self._offline:
            self._wait_for_joint_states()

        current = self._get_eef_pose_from_tf()
        self._print_pose("Current EEF", current.pose)
        src = self._latest_joints or _HOME_JOINTS
        self._print_joints(
            "Current joints",
            self._arm_joint_names,
            [src.get(n, _HOME_JOINTS[n]) for n in self._arm_joint_names],
        )

        target = PoseStamped()
        target.header.frame_id = self._base_frame
        target.header.stamp    = self.get_clock().now().to_msg()
        target.pose.position.x = current.pose.position.x + dx
        target.pose.position.y = current.pose.position.y + dy
        target.pose.position.z = current.pose.position.z + dz
        target.pose.orientation = current.pose.orientation
        self._print_pose("Target  EEF", target.pose)

        return self._plan_and_execute(target, plan_only, planner=planner)

    def move_to_absolute(
        self,
        x: float, y: float, z: float,
        roll: float, pitch: float, yaw: float,
        plan_only: bool = False,
        planner: str = "lin",
    ) -> bool:
        """Move EEF to an absolute pose in the arm base frame.

        Args:
            x, y, z:           Target position in metres.
            roll, pitch, yaw:  Target orientation in radians (intrinsic ZYX RPY).
            plan_only:         Plan only, do not execute.
            planner:           "lin" straight Cartesian | "ptp" shortest joint path
                               | "ompl" RRTConnect | "ompl-opt" RRTstar.
        """
        if not self._offline:
            self._wait_for_joint_states()

        current = self._get_eef_pose_from_tf()
        self._print_pose("Current EEF", current.pose)
        src = self._latest_joints or _HOME_JOINTS
        self._print_joints(
            "Current joints",
            self._arm_joint_names,
            [src.get(n, _HOME_JOINTS[n]) for n in self._arm_joint_names],
        )

        target = PoseStamped()
        target.header.frame_id = self._base_frame
        target.header.stamp    = self.get_clock().now().to_msg()
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        target.pose.orientation = _rpy_to_quaternion(roll, pitch, yaw)
        self._print_pose("Target  EEF", target.pose)

        return self._plan_and_execute(target, plan_only, planner=planner)

    # ── Private ───────────────────────────────────────────────────────────────

    def _print_joints(self, label: str, names: list, values) -> None:
        """Print a joint-angle table in both radians and degrees.

        UR revolute joints can report values outside [-π, π] when the wrist has
        spun more than one full turn.  MoveIt normalises the difference mod 2π
        when checking JointConstraints, so e.g. -5.9373 rad ≡ +0.346 rad and
        both satisfy a constraint centred at +0.60 rad.  An "(≡ …)" note is
        printed whenever the raw value is outside [-π, π].
        """
        print(f"\n  {label}")
        for name, val in zip(names, values):
            short = name.split("_", 1)[-1]  # strip left_/right_ prefix
            # normalise to [-π, π] for readability
            norm = (val + math.pi) % (2 * math.pi) - math.pi
            note = (
                f"  (≡ {norm:+.4f} rad / {math.degrees(norm):+.2f}°)"
                if abs(norm - val) > 1e-3
                else ""
            )
            print(
                f"    {short:<30s}  {val:+.4f} rad  ({math.degrees(val):+8.3f}°){note}"
            )

    def _print_joint_limit_headroom(self) -> None:
        """After a planning failure, show how much room each joint has inside its limit.

        A small headroom means that limit is likely causing the failure.
        """
        source = self._latest_joints or _HOME_JOINTS
        print("\n  Joint limit headroom (why planning may have failed):")
        print(f"    {'joint':<30s}  {'current':>10s}  {'range':>14s}  {'lo room':>9s}  {'hi room':>9s}  note")
        for name in self._arm_joint_names:
            if name not in _JOINT_PATH_LIMITS:
                continue
            lo, hi = _JOINT_PATH_LIMITS[name]
            cur = float(source.get(name, _HOME_JOINTS[name]))
            # normalize cur to be inside [lo, hi] if possible (handles wrap)
            while cur < lo - math.pi:
                cur += 2 * math.pi
            while cur > hi + math.pi:
                cur -= 2 * math.pi
            room_lo = cur - lo
            room_hi = hi - cur
            total   = hi - lo
            short   = name.split("_", 1)[-1]
            flag = " ← TIGHT" if total < 0.70 or min(room_lo, room_hi) < 0.10 else ""
            print(
                f"    {short:<30s}  {math.degrees(cur):>+8.1f}°  "
                f"[{math.degrees(lo):>+6.1f}°,{math.degrees(hi):>+6.1f}°]  "
                f"{math.degrees(room_lo):>+7.1f}°  {math.degrees(room_hi):>+7.1f}°"
                f"{flag}"
            )

    def _print_pose(self, label: str, pose) -> None:
        r, p, y = _quat_to_rpy(pose.orientation)
        print(
            f"\n  {label} ({self._base_frame})\n"
            f"    pos   x={pose.position.x:+.4f}  y={pose.position.y:+.4f}  z={pose.position.z:+.4f}  [m]\n"
            f"    ori   roll={math.degrees(r):+.2f}°  pitch={math.degrees(p):+.2f}°  yaw={math.degrees(y):+.2f}°"
            f"  ({r:+.4f}  {p:+.4f}  {y:+.4f} rad)"
        )

    def _wait_for_joint_states(self) -> None:
        import time
        deadline = time.time() + _JS_WAIT_SEC
        while time.time() < deadline and self._latest_joints is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._latest_joints is None:
            self.get_logger().warn(
                f"No /joint_states after {_JS_WAIT_SEC:.0f}s — "
                "falling back to home joint positions for start state."
            )

    def _get_eef_pose_from_tf(self) -> PoseStamped:
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.05)
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, self._eef_link,
                rclpy.time.Time(),
                timeout=Duration(seconds=_TF_WAIT_SEC),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            raise RuntimeError(
                f"TF lookup failed: {self._base_frame} → {self._eef_link}\n  {exc}"
            ) from exc

        pose = PoseStamped()
        pose.header.frame_id = self._base_frame
        pose.header.stamp    = self.get_clock().now().to_msg()
        t = tf.transform.translation
        pose.pose.position.x  = t.x
        pose.pose.position.y  = t.y
        pose.pose.position.z  = t.z
        pose.pose.orientation = tf.transform.rotation
        return pose

    def _build_start_state(self) -> RobotState:
        source = (
            _HOME_JOINTS
            if (self._offline or self._latest_joints is None)
            else self._latest_joints
        )
        js = JointState()
        js.name     = list(_HOME_JOINTS.keys())
        js.position = [float(source.get(k, _HOME_JOINTS[k])) for k in js.name]
        state = RobotState()
        state.joint_state = js
        return state

    def _build_base_request(self, planner: str = "lin") -> MotionPlanRequest:
        """Build a MotionPlanRequest skeleton.

        planner:
          "lin"      — Pilz LIN: straight line in Cartesian space (default).
          "ptp"      — Pilz PTP: shortest path in joint space (linear joint interp).
          "ompl"     — OMPL RRTConnect: random joint-space search.
          "ompl-opt" — OMPL RRTstar: optimising search (minimises joint-space path
                       length); runs for the full allowed_planning_time.

        Joint path limits are read from the module-level _JOINT_PATH_LIMITS dict
        when _USE_JOINT_PATH_LIMITS is True (ompl / ompl-opt only).
        """
        req = MotionPlanRequest()
        req.group_name                      = self._group
        req.num_planning_attempts           = _NUM_ATTEMPTS
        req.max_velocity_scaling_factor     = _MAX_VEL_SCALE
        req.max_acceleration_scaling_factor = _MAX_ACCEL_SCALE

        if planner in ("lin", "ptp"):
            req.pipeline_id       = "pilz_industrial_motion_planner"
            req.planner_id        = planner.upper()
            req.allowed_planning_time = _PLANNING_TIME_SEC
        elif planner == "ompl-opt":
            req.planner_id        = "RRTstarkConfigDefault"
            req.allowed_planning_time = _PLANNING_TIME_OMPL_OPT
        else:  # "ompl"
            req.planner_id        = "RRTConnectkConfigDefault"
            req.allowed_planning_time = _PLANNING_TIME_SEC

        ws = WorkspaceParameters()
        ws.header.frame_id = self._base_frame
        ws.min_corner.x = ws.min_corner.y = ws.min_corner.z = -_WORKSPACE_HALF_M
        ws.max_corner.x = ws.max_corner.y = ws.max_corner.z =  _WORKSPACE_HALF_M
        req.workspace_parameters = ws
        req.start_state = self._build_start_state()

        if _USE_JOINT_PATH_LIMITS and planner in ("ompl", "ompl-opt"):
            path_c = Constraints()
            print("\n  Joint path limits active:")
            for name in self._arm_joint_names:
                if name not in _JOINT_PATH_LIMITS:
                    continue
                lo, hi = _JOINT_PATH_LIMITS[name]
                center    = (lo + hi) / 2.0
                tolerance = (hi - lo) / 2.0
                jc = JointConstraint()
                jc.joint_name      = name
                jc.position        = center
                jc.tolerance_above = tolerance
                jc.tolerance_below = tolerance
                jc.weight          = 1.0
                path_c.joint_constraints.append(jc)
                short = name.split("_", 1)[-1]
                print(
                    f"    {short:<30s}  [{math.degrees(lo):+.1f}°,  {math.degrees(hi):+.1f}°]"
                )
            req.path_constraints = path_c

        return req

    def _build_cartesian_constraints(self, target: PoseStamped) -> Constraints:
        c = Constraints()
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[_POSITION_TOL_M])
        pos_c = PositionConstraint()
        pos_c.header    = target.header
        pos_c.link_name = self._eef_link
        pos_c.constraint_region = BoundingVolume(
            primitives=[sphere], primitive_poses=[target.pose]
        )
        pos_c.weight = 1.0
        c.position_constraints.append(pos_c)

        ori_c = OrientationConstraint()
        ori_c.header    = target.header
        ori_c.link_name = self._eef_link
        ori_c.orientation = target.pose.orientation
        ori_c.absolute_x_axis_tolerance = _ORIENTATION_TOL_RAD
        ori_c.absolute_y_axis_tolerance = _ORIENTATION_TOL_RAD
        ori_c.absolute_z_axis_tolerance = _ORIENTATION_TOL_RAD
        ori_c.weight = 1.0
        c.orientation_constraints.append(ori_c)
        return c

    def _plan_and_execute(
        self, target: PoseStamped, plan_only: bool, planner: str = "lin"
    ) -> bool:
        return self._run(
            self._build_base_request(planner=planner),
            self._build_cartesian_constraints(target),
            plan_only,
            planner=planner,
        )

    def _run(
        self,
        req: MotionPlanRequest,
        goal_constraints: Constraints,
        plan_only: bool,
        planner: str = "lin",
    ) -> bool:
        req.goal_constraints.append(goal_constraints)

        # ── Step 1: plan ──────────────────────────────────────────────────────
        plan_goal = MoveGroup.Goal()
        plan_goal.planning_options.plan_only = True
        plan_goal.planning_options.planning_scene_diff.is_diff              = True
        plan_goal.planning_options.planning_scene_diff.robot_state.is_diff  = True
        plan_goal.request = req

        _labels = {
            "lin":      "Pilz LIN (straight Cartesian)",
            "ptp":      "Pilz PTP (shortest joint path)",
            "ompl":     "OMPL RRTConnect",
            "ompl-opt": "OMPL RRTstar (optimising)",
        }
        planner_label = _labels.get(planner, planner)
        print(f"\n  [{self._group}] Planning with {planner_label}…")
        send_future = self._client.send_goal_async(plan_goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            print("  ERROR: move_group rejected the planning goal.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val != 1:
            label = _MOVEIT_ERROR_LABELS.get(
                result.error_code.val, f"code={result.error_code.val}"
            )
            print(f"  ERROR: planning FAILED ({label})")
            if _USE_JOINT_PATH_LIMITS and planner in ("ompl", "ompl-opt"):
                self._print_joint_limit_headroom()
            return False

        trajectory = result.planned_trajectory
        print("  Planning SUCCESS.")

        jt = trajectory.joint_trajectory
        if jt.points:
            self._print_joints(
                "Target  joints (planned)",
                jt.joint_names,
                jt.points[-1].positions,
            )

        # ── Step 2: preview in RViz + EEF path marker ─────────────────────────
        disp = DisplayTrajectory()
        disp.trajectory_start = req.start_state
        disp.trajectory       = [trajectory]
        self._display_pub.publish(disp)
        self._publish_trajectory_markers(trajectory)

        if plan_only:
            print("  Plan-only mode — trajectory shown in RViz. Not executing.")
            return True

        # ── Step 3: confirm ───────────────────────────────────────────────────
        try:
            input("\n  Press Enter to execute on the robot, or Ctrl+C to abort… ")
        except (KeyboardInterrupt, EOFError):
            print("\n  Aborted.")
            return False

        # ── Step 4: execute ───────────────────────────────────────────────────
        return self._execute_trajectory(trajectory)

    def _execute_trajectory(self, trajectory) -> bool:
        if not self._execute_client.wait_for_server(timeout_sec=5.0):
            print(f"  ERROR: '{_EXECUTE_ACTION}' not available.")
            return False

        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = trajectory

        print("  Executing…")
        send_future = self._execute_client.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self, send_future)
        gh = send_future.result()

        if not gh.accepted:
            print("  ERROR: execution goal rejected by controller manager.")
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val == 1:
            print("  Execution SUCCESS — robot reached the target.")
            return True

        label = _MOVEIT_ERROR_LABELS.get(result.error_code.val, f"code={result.error_code.val}")
        print(f"  ERROR: execution FAILED ({label})")
        return False

    # ── Trajectory markers ────────────────────────────────────────────────────

    def _publish_trajectory_markers(self, trajectory) -> None:
        joint_traj = trajectory.joint_trajectory
        if not joint_traj.points:
            return
        if not self._fk_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"'{_FK_SERVICE}' not available — skipping markers.")
            return

        # Clear old markers
        delete_all = Marker()
        delete_all.header.frame_id = self._base_frame
        delete_all.header.stamp    = self.get_clock().now().to_msg()
        delete_all.action          = Marker.DELETEALL
        self._marker_pub.publish(delete_all)

        n = len(joint_traj.points)
        indices = [int(i * (n - 1) / (_MARKER_MAX_POINTS - 1)) for i in range(min(_MARKER_MAX_POINTS, n))]
        if indices[-1] != n - 1:
            indices[-1] = n - 1

        points: list[Point] = []
        for idx in indices:
            pt  = joint_traj.points[idx]
            fk  = self._call_fk(list(joint_traj.joint_names), list(pt.positions))
            if fk is not None:
                points.append(fk.pose.position)

        if not points:
            return

        now      = self.get_clock().now().to_msg()
        lifetime = rclpy.duration.Duration(seconds=_MARKER_LIFETIME_SEC).to_msg()

        line           = Marker()
        line.header.frame_id = self._base_frame
        line.header.stamp    = now
        line.ns        = "eef_path"
        line.id        = 0
        line.type      = Marker.LINE_STRIP
        line.action    = Marker.ADD
        line.scale.x   = 0.015
        line.color     = _color(0.0, 0.85, 1.0)
        line.lifetime  = lifetime
        line.points    = points
        self._marker_pub.publish(line)

        for mid, pt, col in [(0, points[0],  _color(0.0, 1.0, 0.0)),
                              (1, points[-1], _color(1.0, 0.2, 0.0))]:
            s = Marker()
            s.header   = line.header
            s.ns       = "eef_waypoints"
            s.id       = mid
            s.type     = Marker.SPHERE
            s.action   = Marker.ADD
            s.pose.position   = pt
            s.pose.orientation.w = 1.0
            s.scale.x = s.scale.y = s.scale.z = 0.040
            s.color    = col
            s.lifetime = lifetime
            self._marker_pub.publish(s)

        print(f"  Markers published on '{_MARKER_TOPIC}' ({len(points)} pts, {_MARKER_LIFETIME_SEC:.0f}s).")

    def _call_fk(self, joint_names: list, joint_positions: list):
        req = GetPositionFK.Request()
        req.header.frame_id       = self._base_frame
        req.fk_link_names         = [self._eef_link]
        js = JointState()
        js.name                   = joint_names
        js.position               = joint_positions
        req.robot_state.joint_state = js
        future = self._fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=_FK_TIMEOUT_SEC)
        if not future.done():
            return None
        resp = future.result()
        if resp is None or resp.error_code.val != 1:
            return None
        return resp.pose_stamped[0] if resp.pose_stamped else None


# ── Entry point ───────────────────────────────────────────────────────────────


def main(argv=None) -> None:
    # ╔══════════════════════════════════════════════════════════════════════╗
    # ║  HARDCODED TARGET — edit these values to change the default motion.  ║
    # ║  These are used when you run:                                         ║
    # ║    ros2 run aruco_moveit_planner move_arm                            ║
    # ║  without any CLI flags.                                               ║
    # ║                                                                       ║
    # ║  Frame: right_base (right arm) or left_base (left arm).              ║
    # ╚══════════════════════════════════════════════════════════════════════╝

    # --- Absolute target pose (set USE_TARGET = True to use this) -----------
    USE_TARGET = False          # ← change to True to send an absolute pose
    _X,    _Y,    _Z     = 0.3,  0.1,  0.8    # metres
    _ROLL, _PITCH, _YAW  = 0.0,  1.57, 0.0    # radians

    # --- Delta move (used when USE_TARGET = False) ---------------------------
    _DX, _DY, _DZ = 0.30, -0.10, 0.20            # metres  (default: +10 cm Z)

    # ════════════════════════════════════════════════════════════════════════

    cli_args = rclpy.utilities.remove_ros_args(argv or sys.argv[1:])

    parser = argparse.ArgumentParser(
        description="Plan then (after Enter) execute a single arm Cartesian move."
    )
    parser.add_argument("--arm",      choices=["right", "left"], default="right")
    parser.add_argument("--offline",  action="store_true",
                        help="Use home joints as start state (fake hw).")
    parser.add_argument("--plan-only", action="store_true", dest="plan_only",
                        help="Plan only — show in RViz, no execution prompt.")
    parser.add_argument(
        "--planner",
        choices=["lin", "ptp", "ompl", "ompl-opt"],
        default="ptp",
        help=(
            "ptp      (default) — Pilz PTP: shortest path in joint space, "
            "minimises joint rotation — best for efficient moves.\n"
            "lin                — Pilz LIN: straight line in Cartesian space.\n"
            "ompl               — OMPL RRTConnect: random joint-space search "
            "(use when LIN/PTP fail at singularities).\n"
            "ompl-opt           — OMPL RRTstar: optimising search, runs for "
            f"{_PLANNING_TIME_OMPL_OPT:.0f}s and returns the best path found."
        ),
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--home",   action="store_true",
                      help="Move to SRDF home configuration.")
    mode.add_argument("--delta",  type=float, nargs=3, metavar=("DX","DY","DZ"),
                      help="Translation delta in metres (orientation unchanged).")
    mode.add_argument("--target", type=float, nargs=6,
                      metavar=("X","Y","Z","ROLL","PITCH","YAW"),
                      help="Absolute pose in arm base frame (m + rad RPY).")

    parsed = parser.parse_args(cli_args)

    rclpy.init(args=argv)
    success = False
    try:
        node = MoveArmClient(arm=parsed.arm, offline=parsed.offline)

        p = parsed.planner

        if parsed.home:
            success = node.move_to_home(plan_only=parsed.plan_only)

        elif parsed.target is not None:
            x, y, z, roll, pitch, yaw = parsed.target
            success = node.move_to_absolute(
                x, y, z, roll, pitch, yaw,
                plan_only=parsed.plan_only, planner=p,
            )

        elif parsed.delta is not None:
            dx, dy, dz = parsed.delta
            success = node.move_delta(
                dx, dy, dz, plan_only=parsed.plan_only, planner=p,
            )

        else:
            # No CLI flag given — use the HARDCODED TARGET block above
            if USE_TARGET:
                success = node.move_to_absolute(
                    _X, _Y, _Z, _ROLL, _PITCH, _YAW,
                    plan_only=parsed.plan_only, planner=p,
                )
            else:
                success = node.move_delta(
                    _DX, _DY, _DZ, plan_only=parsed.plan_only, planner=p,
                )

    except RuntimeError as exc:
        print(f"\n  ERROR: {exc}", file=sys.stderr)
    finally:
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
