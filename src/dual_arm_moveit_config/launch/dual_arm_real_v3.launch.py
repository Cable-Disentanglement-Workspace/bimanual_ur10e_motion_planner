"""
dual_arm_real_v3.launch.py
Brings up both real UR10e arms: hardware, controllers, MoveIt and (optionally) RViz.

Inherited from v2:
  - Official ur_description URDF (correct kinematics from the real robots).
  - A single controller_manager for both arms, so there is no namespace conflict.

New in v3:
  - Startup is event-driven instead of time-driven. v2 used fixed TimerActions
    (3/5/15/18 s) that were both wasteful on a good day and too short on a bad one.
    Each stage now starts the moment the previous one reports done:

        ros2_control_node + broadcasters          (t=0)
          -> joint_state_broadcaster exits 0
        left_arm_controller, right_arm_controller
          -> right_arm_controller exits 0
        move_group + rviz

    A spawner is transient: it waits for /controller_manager, activates its
    controller, then exits 0 -- that exit is the readiness signal. Spawners wait
    indefinitely (--controller-manager-timeout defaults to 0), so no stage can
    fire early, however long the robots take to come up.

    Consequence: if an arm is powered off, e-stopped or unreachable, the launch
    WAITS instead of failing fast. If neither "-> " progress line appears, check
    the robots and the network before suspecting this file.

    RViz starts alongside move_group rather than after it: move_group is
    long-running and never exits, so there is no exit event to chain it off.

  - use_rviz launch argument (default true); set false for headless planning.

Real vs fake hardware is decided entirely by the URDF, not by the nodes below.
This file passes no use_fake_hardware argument to xacro, so it defaults to false
and ros2_control_node loads ur_robot_driver's hardware interface, which opens the
RTDE connection to 192.168.1.10 / .20. For the offline version (mock hardware, no
robots on the network) use dual_arm_fake_v2.launch.py.

The two urscript_interface nodes are a side channel for sending raw URScript,
independent of the RTDE/MoveIt path; removing them would not affect the connection.

Usage:
    ros2 launch dual_arm_moveit_config dual_arm_real_v3.launch.py
    ros2 launch dual_arm_moveit_config dual_arm_real_v3.launch.py use_rviz:=false

Verify:
    ros2 control list_controllers     # seven controllers, all 'active'
    ros2 topic hz /joint_states       # live data from the real arms
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg = get_package_share_directory("dual_arm_moveit_config")

    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            os.path.join(pkg, "config", "dual_arm_real_v2.urdf.xacro"),
        ]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Publish gripper finger joints at 0 (open)
    gripper_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="gripper_state_publisher",
        parameters=[{
            "source_list": ["joint_states"],
            "rate": 50,
        }],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            os.path.join(pkg, "config", "ros2_controllers.yaml"),
        ],
        output="screen",
    )

    def spawner(controller):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "--controller-manager", "/controller_manager"],
            output="screen",
        )

    jsb = spawner("joint_state_broadcaster")
    left_arm = spawner("left_arm_controller")
    right_arm = spawner("right_arm_controller")

    move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, "launch", "move_group.launch.py")
            )
        )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "moveit_rviz.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    right_urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        name="right_urscript_interface",
        parameters=[{"robot_ip": "192.168.1.20"}],
        output="screen",
    )

    left_urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        name="left_urscript_interface",
        parameters=[{"robot_ip": "192.168.1.10"}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Launch RViz with the MoveIt panels. Set false for headless "
                        "planning (move_group and controllers still come up).",
        ),
        rsp_node,
        gripper_state_publisher,
        ros2_control_node,
        right_urscript_interface,
        left_urscript_interface,
        jsb,
        spawner("left_io_and_status_controller"),
        spawner("right_io_and_status_controller"),
        spawner("left_speed_scaling_state_broadcaster"),
        spawner("right_speed_scaling_state_broadcaster"),
        RegisterEventHandler(OnProcessExit(
            target_action=jsb,
            on_exit=[LogInfo(msg="joint_state_broadcaster active -> arm controllers"),
                     left_arm, right_arm],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=right_arm,
            on_exit=[LogInfo(msg="arm controllers active -> move_group + rviz"),
                     move_group, rviz],
        )),
    ])
