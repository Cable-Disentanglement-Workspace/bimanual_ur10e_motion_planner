"""ArUco marker pose acquisition from a ROS 2 topic or a JSON file.

The JSON schema mirrors the ROS 2 ``geometry_msgs/PoseStamped`` message::

    {
        "header": {"frame_id": "zed_left_camera_frame"},
        "pose": {
            "position":    {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        }
    }
"""

import json
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped


def read_from_json(json_path: str) -> PoseStamped:
    """Load a ``PoseStamped`` from a JSON file that mirrors the ROS 2 message layout.

    Args:
        json_path: Absolute or relative path to the JSON file.

    Returns:
        Populated ``PoseStamped`` message.

    Raises:
        FileNotFoundError: If the file does not exist.
        KeyError: If required fields are absent from the JSON.
    """
    path = Path(json_path)
    if not path.exists():
        raise FileNotFoundError(f"Pose JSON not found: {json_path}")

    with path.open() as fh:
        data = json.load(fh)

    msg = PoseStamped()
    msg.header.frame_id = data["header"]["frame_id"]

    pos = data["pose"]["position"]
    msg.pose.position.x = float(pos["x"])
    msg.pose.position.y = float(pos["y"])
    msg.pose.position.z = float(pos["z"])

    ori = data["pose"]["orientation"]
    msg.pose.orientation.x = float(ori["x"])
    msg.pose.orientation.y = float(ori["y"])
    msg.pose.orientation.z = float(ori["z"])
    msg.pose.orientation.w = float(ori["w"])

    return msg


class PoseTopicReader(Node):
    """Single-shot subscriber that captures one ``PoseStamped`` from a topic.

    Spin the node (via :meth:`wait_for_pose`) until the first message arrives,
    then stop.  The node should be destroyed by the caller after use.

    Args:
        topic: Topic name to subscribe to.
        timeout_sec: Maximum seconds to wait before raising ``TimeoutError``.
    """

    def __init__(self, topic: str, timeout_sec: float = 10.0) -> None:
        super().__init__("aruco_pose_reader")
        self._result: Optional[PoseStamped] = None
        self._timeout_sec = timeout_sec

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(PoseStamped, topic, self._callback, qos)
        self.get_logger().info(
            f"Waiting for marker pose on '{topic}' (timeout={timeout_sec:.1f}s)…"
        )

    def _callback(self, msg: PoseStamped) -> None:
        """Store the first received message and ignore subsequent ones."""
        if self._result is None:
            self._result = msg

    def wait_for_pose(self) -> PoseStamped:
        """Block until a message arrives or the timeout expires.

        Returns:
            The first received ``PoseStamped``.

        Raises:
            TimeoutError: If no message arrives within ``timeout_sec``.
        """
        deadline = time.monotonic() + self._timeout_sec
        while rclpy.ok() and self._result is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"No marker pose received within {self._timeout_sec:.1f}s. "
                    "Ensure aruco_ros is running and detecting a marker."
                )

        pose = self._result
        self.get_logger().info(
            f"Received marker pose in frame '{pose.header.frame_id}': "
            f"pos=({pose.pose.position.x:.4f}, "
            f"{pose.pose.position.y:.4f}, "
            f"{pose.pose.position.z:.4f})"
        )
        return pose
