"""
dual_arm_fake.launch.py
Offline / no-robot version of dual_arm_real_v2.launch.py.

Both UR10e arms are driven by mock_components/GenericSystem (use_fake_hardware:=true),
so the full MoveIt stack (robot model, move_group, RViz) comes up without any physical
robot on the network. You can plan and "execute" trajectories in RViz; the mock hardware
simply echoes the commanded joint states back. Use this for visualization and planning.

For the real robots, use dual_arm_real_v2.launch.py instead.
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
            " use_fake_hardware:=true",
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

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Launch RViz with the MoveIt panels. Set false for headless "
                        "planning (move_group and controllers still come up).",
        ),
        rsp_node,
        gripper_state_publisher,
        ros2_control_node,
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
