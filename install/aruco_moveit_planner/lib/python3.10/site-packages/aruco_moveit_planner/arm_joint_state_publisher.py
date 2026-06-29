"""Publishes arm joint states at home positions when hardware is offline.

Publishes only the 12 arm joints (6 left + 6 right) to ``/joint_states`` at
50 Hz.  The ``gripper_state_publisher`` already running in
``dual_arm_real_v2.launch.py`` reads from ``/joint_states`` and merges the
gripper finger joints, giving ``robot_state_publisher`` a complete joint
state so RViz can render the robot at its home configuration.

Designed for use when the physical UR10e robots are **not** connected.
When hardware connects and ``joint_state_broadcaster`` activates, stop
this node (or set ``use_fake_joints:=false`` in the launch file) to avoid
a duplicate-publisher conflict on ``/joint_states``.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Joint values match the SRDF left_home / right_home named states.
_HOME_JOINTS: dict = {
    "left_shoulder_pan_joint":   -0.5842,
    "left_shoulder_lift_joint":  -1.4352,
    "left_elbow_joint":          -2.4564,
    "left_wrist_1_joint":         0.5894,
    "left_wrist_2_joint":        -4.1043,
    "left_wrist_3_joint":        -1.7400,
    "right_shoulder_pan_joint":  -2.5287,
    "right_shoulder_lift_joint": -1.9195,
    "right_elbow_joint":          2.5167,
    "right_wrist_1_joint":       -3.4194,
    "right_wrist_2_joint":       -2.0268,
    "right_wrist_3_joint":        0.2502,
}

_PUBLISH_RATE_HZ: float = 50.0


class ArmJointStatePublisher(Node):
    """Continuously publishes static arm joint positions to ``/joint_states``.

    Publishing at 50 Hz keeps ``robot_state_publisher`` and the MoveIt
    planning scene monitor well-fed without saturating the topic.
    """

    def __init__(self) -> None:
        super().__init__("arm_joint_state_publisher")
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(1.0 / _PUBLISH_RATE_HZ, self._publish)
        self.get_logger().info(
            f"Publishing {len(_HOME_JOINTS)} arm joints at home positions "
            f"on /joint_states at {_PUBLISH_RATE_HZ:.0f} Hz. "
            "Stop this node when real hardware connects."
        )

    def _publish(self) -> None:
        """Emit one ``JointState`` message with home joint positions."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(_HOME_JOINTS.keys())
        msg.position = list(_HOME_JOINTS.values())
        self._pub.publish(msg)


def main(argv=None) -> None:
    """Entry point for the arm_joint_state_publisher executable."""
    rclpy.init(args=argv)
    node = ArmJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
