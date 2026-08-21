from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

# Eye-in-hand calibration for the LEFT arm's wrist D405.
#
# Unlike the ZED (eye_on_base: fixed camera, marker on the moving gripper),
# this is eye_in_hand: the camera moves with the arm, and the ArUco marker
# must be FIXED in the environment (e.g. taped to the table) so the wrist
# camera sees it from a different angle at every jogged pose.
#
# Physical camera: user-confirmed (2026-08-21) that IP .10 = left arm, which
# resolves the robot1/robot2-vs-left/right contradiction: robot1 (default
# robot1_ip=.10 in page6_deployment.py) = left arm, robot2 (.20) = right arm.
# il_sandbox/page6_deployment.py's tooltips ("robot1_ip = right arm") are
# stale/wrong. Consequently, the D405 serial comments in
# il_sandbox/real_robot/launch/real_robot_bringup.launch.py are also backwards:
# serial 230322274698 (commented "right wrist D405 (robot1)") is actually the
# LEFT arm's wrist camera — that's the one this file calibrates.
#
# Still verify before running:
#   1. The wrist camera's actual optical TF frame id — run
#      `ros2 topic echo /camera_left/camera_left/color/camera_info --field header.frame_id`
#      (or `ros2 run tf2_ros tf2_echo` once the camera node is up) and pass it
#      via the tracking_base_frame launch argument if it differs from the
#      default guessed below.

def generate_launch_description():
    easy_handeye2_dir = get_package_share_directory('easy_handeye2')

    tracking_base_frame_arg = DeclareLaunchArgument(
        'tracking_base_frame',
        default_value='camera_left_color_optical_frame',
        description=(
            "Wrist D405 (left arm) optical TF frame. VERIFY this against the "
            "running camera node before calibrating -- realsense2_camera's "
            "actual frame_id depends on the node/camera name it was launched "
            "with (see real_robot_bringup.launch.py's camera_left_node)."
        ),
    )

    return LaunchDescription([
        tracking_base_frame_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(easy_handeye2_dir, 'launch', 'calibrate.launch.py')),
            launch_arguments={
                'calibration_type': 'eye_in_hand',
                'name': 'd405_left_wrist_calib',
                'robot_base_frame': 'left_base',
                'robot_effector_frame': 'left_tool0',
                'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
                'tracking_marker_frame': 'aruco_marker_frame',
                'freehand_robot_movement': 'true',
            }.items(),
        )
    ])
