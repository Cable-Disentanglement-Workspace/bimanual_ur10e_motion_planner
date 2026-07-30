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
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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

    def spawner(controller, delay):
        return TimerAction(period=delay, actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ])

    move_group = TimerAction(
        period=15.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, "launch", "move_group.launch.py")
            )
        )]
    )

    rviz = TimerAction(
        period=18.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, "launch", "moveit_rviz.launch.py")
            )
        )]
    )

    return LaunchDescription([
        rsp_node,
        gripper_state_publisher,
        ros2_control_node,
        spawner("joint_state_broadcaster",               3.0),
        spawner("left_io_and_status_controller",         3.0),
        spawner("right_io_and_status_controller",        3.0),
        spawner("left_speed_scaling_state_broadcaster",  3.0),
        spawner("right_speed_scaling_state_broadcaster", 3.0),
        spawner("left_arm_controller",                   5.0),
        spawner("right_arm_controller",                  5.0),
        move_group,
        rviz,
    ])
