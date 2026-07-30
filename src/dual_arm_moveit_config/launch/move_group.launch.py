from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("dual_arm", package_name="dual_arm_moveit_config")
        .robot_description(file_path="config/dual_arm_real_v2.urdf.xacro")
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "monitor_dynamics": False,
            },
        ],
    )

    return LaunchDescription([move_group_node])
