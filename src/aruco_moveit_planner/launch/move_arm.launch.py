"""Launch file for move_arm — plan/execute a single arm motion.

Arguments
---------
arm             : right | left                  (default: right)
offline         : true | false                  (default: false)
plan_only       : true | false                  (default: false)
planner         : lin | ptp | ompl | ompl-opt   (default: lin)
max_joint_delta : radians, e.g. 0.5             (default: "", disabled)

Motion mode (mutually exclusive; default is delta):
  home     : true | false          Reset to SRDF home pose
  dx dy dz : metres                Translation delta (default: 0 0 0.10)
  x y z    : metres                )
  roll     :                       ) Absolute target pose in arm base frame.
  pitch    : radians               ) Overrides dx/dy/dz when x is set.
  yaw      :                       )

Examples
--------
# Offline +10 cm Z, plan only:
  ros2 launch aruco_moveit_planner move_arm.launch.py offline:=true plan_only:=true

# Online absolute target for right arm:
  ros2 launch aruco_moveit_planner move_arm.launch.py \\
      x:=0.3 y:=0.1 z:=0.8 roll:=0.0 pitch:=1.57 yaw:=0.0

# Reset right arm to home (real robot):
  ros2 launch aruco_moveit_planner move_arm.launch.py home:=true

# OMPL RRTstar with ±30° joint limit:
  ros2 launch aruco_moveit_planner move_arm.launch.py \\
      planner:=ompl-opt max_joint_delta:=0.524
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_cmd_args(context, *args, **kwargs):
    arm             = LaunchConfiguration("arm").perform(context)
    offline         = LaunchConfiguration("offline").perform(context)
    plan_only       = LaunchConfiguration("plan_only").perform(context)
    planner         = LaunchConfiguration("planner").perform(context)
    max_joint_delta = LaunchConfiguration("max_joint_delta").perform(context)
    home            = LaunchConfiguration("home").perform(context)
    dx              = LaunchConfiguration("dx").perform(context)
    dy              = LaunchConfiguration("dy").perform(context)
    dz              = LaunchConfiguration("dz").perform(context)
    x               = LaunchConfiguration("x").perform(context)
    y               = LaunchConfiguration("y").perform(context)
    z               = LaunchConfiguration("z").perform(context)
    roll            = LaunchConfiguration("roll").perform(context)
    pitch           = LaunchConfiguration("pitch").perform(context)
    yaw             = LaunchConfiguration("yaw").perform(context)

    cmd_args = ["--arm", arm, "--planner", planner]

    if offline.lower() == "true":
        cmd_args.append("--offline")

    if plan_only.lower() == "true":
        cmd_args.append("--plan-only")

    if max_joint_delta != "":
        cmd_args += ["--max-joint-delta", max_joint_delta]

    # Motion mode: --home > --target (when x is provided) > --delta
    if home.lower() == "true":
        cmd_args.append("--home")
    elif x != "":
        cmd_args += ["--target", x, y, z, roll, pitch, yaw]
    else:
        cmd_args += ["--delta", dx, dy, dz]

    node = Node(
        package="aruco_moveit_planner",
        executable="move_arm",
        name="move_arm",
        output="screen",
        arguments=cmd_args,
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arm",       default_value="right",
                              description="right or left"),
        DeclareLaunchArgument("offline",   default_value="false",
                              description="Use home joints as start state (fake hw)"),
        DeclareLaunchArgument("plan_only", default_value="false",
                              description="Plan only, do not execute"),
        DeclareLaunchArgument("planner",   default_value="ptp",
                              description="lin | ptp | ompl | ompl-opt"),
        DeclareLaunchArgument("max_joint_delta", default_value="",
                              description="Per-joint path constraint in radians (ompl/ompl-opt only)"),
        DeclareLaunchArgument("home",      default_value="false",
                              description="Move to SRDF home pose"),
        DeclareLaunchArgument("dx",        default_value="0.0",
                              description="Delta X in metres"),
        DeclareLaunchArgument("dy",        default_value="0.0",
                              description="Delta Y in metres"),
        DeclareLaunchArgument("dz",        default_value="0.10",
                              description="Delta Z in metres (default +10 cm)"),
        DeclareLaunchArgument("x",         default_value="",
                              description="Absolute X in metres (arm base frame)"),
        DeclareLaunchArgument("y",         default_value="0.0",
                              description="Absolute Y in metres"),
        DeclareLaunchArgument("z",         default_value="0.0",
                              description="Absolute Z in metres"),
        DeclareLaunchArgument("roll",      default_value="0.0",
                              description="Roll in radians"),
        DeclareLaunchArgument("pitch",     default_value="0.0",
                              description="Pitch in radians"),
        DeclareLaunchArgument("yaw",       default_value="0.0",
                              description="Yaw in radians"),
        OpaqueFunction(function=_build_cmd_args),
    ])
