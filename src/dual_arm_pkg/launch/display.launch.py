import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HOME_POSITIONS = {
    "left_shoulder_pan_joint":    -0.5842,
    "left_shoulder_lift_joint":   -1.4352,
    "left_elbow_joint":           -2.4564,
    "left_wrist_1_joint":          0.5894,
    "left_wrist_2_joint":         -4.1043,
    "left_wrist_3_joint":         -1.7400,
    "right_shoulder_pan_joint":   -2.5287,
    "right_shoulder_lift_joint":  -1.9195,
    "right_elbow_joint":           2.5167,
    "right_wrist_1_joint":        -3.4194,
    "right_wrist_2_joint":        -2.0268,
    "right_wrist_3_joint":         0.2502,
    "left_finger_joint":           0.0,
    "right_finger_joint":          0.0,
}

def generate_launch_description():
    pkg = get_package_share_directory("dual_arm_pkg")
    xacro_file = os.path.join(pkg, "urdf", "dual_arm.urdf.xacro")

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file]),
        value_type=str
    )

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            parameters=[{"zeros": HOME_POSITIONS}],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
        ),
    ])