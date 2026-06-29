"""Launch the aruco_moveit_planner node.

Prerequisites
-------------
``dual_arm_real_v2.launch.py`` must be running first so that both arms are
visible in RViz and the move_group action server is available::

    ros2 launch dual_arm_moveit_config dual_arm_real_v2.launch.py

Then, in a second terminal::

    # Read from live ArUco topic (default)
    ros2 launch aruco_moveit_planner plan_to_aruco.launch.py

    # Read from JSON file
    ros2 launch aruco_moveit_planner plan_to_aruco.launch.py \\
        source:=json \\
        json_path:=/home/rosi/ws_dual_arm/src/aruco_moveit_planner/config/sample_aruco_pose.json

    # When physical hardware IS connected, disable the fake joint publisher:
    ros2 launch aruco_moveit_planner plan_to_aruco.launch.py \\
        use_fake_joints:=false

Launch arguments
----------------
source           : 'topic' (default) or 'json'
json_path        : absolute path to the PoseStamped JSON file (source:=json only)
topic            : ROS 2 topic for the ArUco PoseStamped  (default: /aruco_single/pose)
timeout          : seconds to wait for a topic message     (default: 10.0)
calib_path       : path to the easy_handeye2 .calib file
use_fake_joints  : publish home joint positions when hardware is offline (default: true)

Notes on use_fake_joints
------------------------
When ``use_fake_joints:=true`` (the default), this launch file starts an
``arm_joint_state_publisher`` node that emits the 12 UR10e arm joints at their
SRDF home positions to ``/joint_states`` at 50 Hz.  The ``gripper_state_publisher``
already running in ``dual_arm_real_v2.launch.py`` reads from ``/joint_states``,
adds the gripper finger joints, and republishes — giving ``robot_state_publisher``
a complete joint state so RViz renders both arms at their home configuration.

Set ``use_fake_joints:=false`` when the physical UR10e robots are connected and
``joint_state_broadcaster`` is active — otherwise the two publishers on
``/joint_states`` will conflict.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_DEFAULT_CALIB = (
    "/home/rosi/.ros2/easy_handeye2/calibrations/zed_left_arm_calib.calib"
)


def generate_launch_description() -> LaunchDescription:
    """Return the LaunchDescription for the plan_to_aruco node."""
    return LaunchDescription([
        DeclareLaunchArgument(
            "source",
            default_value="topic",
            description="Pose source: 'topic' or 'json'.",
        ),
        DeclareLaunchArgument(
            "json_path",
            default_value="",
            description="Absolute path to PoseStamped JSON file (source:=json only).",
        ),
        DeclareLaunchArgument(
            "topic",
            default_value="/aruco_single/pose",
            description="ROS 2 topic for the ArUco marker PoseStamped.",
        ),
        DeclareLaunchArgument(
            "timeout",
            default_value="10.0",
            description="Seconds to wait for a topic message before aborting.",
        ),
        DeclareLaunchArgument(
            "calib_path",
            default_value=_DEFAULT_CALIB,
            description="Path to the easy_handeye2 .calib file.",
        ),
        DeclareLaunchArgument(
            "use_fake_joints",
            default_value="true",
            description=(
                "Publish arm joints at home positions when hardware is offline. "
                "Set false when physical robots are connected."
            ),
        ),

        # Publishes 12 arm joints at home positions to /joint_states so that:
        # (a) robot_state_publisher computes TF → RViz shows the robot correctly.
        # (b) gripper_state_publisher in dual_arm_real_v2.launch.py merges
        #     gripper joints → RSP gets a complete joint state.
        # Only starts when use_fake_joints:=true (hardware not connected).
        Node(
            package="aruco_moveit_planner",
            executable="arm_joint_state_publisher",
            name="arm_joint_state_publisher",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_fake_joints")),
        ),

        Node(
            package="aruco_moveit_planner",
            executable="plan_to_aruco",
            name="aruco_moveit_planner",
            output="screen",
            arguments=[
                "--source",     LaunchConfiguration("source"),
                "--json-path",  LaunchConfiguration("json_path"),
                "--topic",      LaunchConfiguration("topic"),
                "--timeout",    LaunchConfiguration("timeout"),
                "--calib-path", LaunchConfiguration("calib_path"),
            ],
        ),
    ])
