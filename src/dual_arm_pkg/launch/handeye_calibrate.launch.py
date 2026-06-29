from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    easy_handeye2_dir = get_package_share_directory('easy_handeye2')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(easy_handeye2_dir, 'launch', 'calibrate.launch.py')),
            launch_arguments={
                'calibration_type': 'eye_on_base',
                'name': 'zed_left_arm_calib',
                'robot_base_frame': 'left_base',
                'robot_effector_frame': 'left_tool0',
                'tracking_base_frame': 'zed_left_camera_frame',
                'tracking_marker_frame': 'aruco_marker_frame',
                'freehand_robot_movement': 'true',
            }.items(),
        )
    ])
