"""Hand-eye calibration loading and coordinate-frame transformations.

Conventions
-----------
* All 4×4 matrices are homogeneous transforms: ``T_A_B`` maps a point expressed
  in frame **B** into frame **A** (i.e. ``p_A = T_A_B @ p_B``).
* The ArUco marker's **+Z** axis (OpenCV / aruco_ros convention) points **outward
  from the marker face toward the camera**.
* The ``left_tcp`` goal orientation is the marker frame rotated **180° around the
  marker X-axis**, which makes the TCP **+Z** anti-parallel to the marker **+Z**
  (gripper faces the marker squarely from the camera side).
"""

from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import PoseStamped


# 180° rotation around X: flips Y and Z — used to orient the TCP facing the marker.
_Rx180: np.ndarray = np.array(
    [[1, 0, 0],
     [0, -1, 0],
     [0, 0, -1]],
    dtype=float,
)


def load_calibration(calib_path: str) -> np.ndarray:
    """Parse an easy_handeye2 ``.calib`` YAML file and return ``T_base_camera``.

    Args:
        calib_path: Path to the ``.calib`` file produced by easy_handeye2.

    Returns:
        4×4 homogeneous transform mapping points from the camera frame
        (``zed_left_camera_frame``) into the robot base frame (``left_base``).

    Raises:
        FileNotFoundError: If the file is missing.
        KeyError: If expected YAML keys are absent.
    """
    path = Path(calib_path)
    if not path.exists():
        raise FileNotFoundError(f"Calibration file not found: {calib_path}")

    with path.open() as fh:
        data = yaml.safe_load(fh)

    t = data["transform"]["translation"]
    r = data["transform"]["rotation"]

    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([r["x"], r["y"], r["z"], r["w"]]).as_matrix()
    T[:3, 3] = [t["x"], t["y"], t["z"]]
    return T


def _pose_to_matrix(pose: PoseStamped) -> np.ndarray:
    """Convert a ``PoseStamped`` to a 4×4 homogeneous transform."""
    p = pose.pose.position
    o = pose.pose.orientation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def _matrix_to_pose(T: np.ndarray, frame_id: str) -> PoseStamped:
    """Convert a 4×4 homogeneous transform to a ``PoseStamped``.

    Args:
        T: 4×4 homogeneous transform.
        frame_id: The ``header.frame_id`` to stamp on the returned message.

    Returns:
        ``PoseStamped`` representing the same pose.
    """
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(T[0, 3])
    pose.pose.position.y = float(T[1, 3])
    pose.pose.position.z = float(T[2, 3])
    qx, qy, qz, qw = Rotation.from_matrix(T[:3, :3]).as_quat()
    pose.pose.orientation.x = float(qx)
    pose.pose.orientation.y = float(qy)
    pose.pose.orientation.z = float(qz)
    pose.pose.orientation.w = float(qw)
    return pose


def transform_marker_to_base(
    T_base_camera: np.ndarray,
    marker_pose_camera: PoseStamped,
) -> PoseStamped:
    """Express the ArUco marker pose in the robot base frame.

    Applies the eye-on-base calibration transform::

        T_base_marker = T_base_camera @ T_camera_marker

    Args:
        T_base_camera: 4×4 transform from camera to robot base (from calib file).
        marker_pose_camera: Marker pose expressed in the camera frame.

    Returns:
        Marker pose expressed in the ``left_base`` robot frame.
    """
    T_camera_marker = _pose_to_matrix(marker_pose_camera)
    T_base_marker = T_base_camera @ T_camera_marker
    return _matrix_to_pose(T_base_marker, frame_id="left_base")


def compute_tcp_goal(marker_pose_base: PoseStamped) -> PoseStamped:
    """Compute the ``left_tcp`` goal pose from the marker pose in the base frame.

    The TCP position coincides with the marker centre.  The TCP orientation
    is obtained by rotating the marker frame **180° around its own X-axis**,
    which makes the TCP ``+Z`` anti-parallel to the marker ``+Z``.  This
    orients the gripper to face the marker squarely from the camera side::

        R_tcp = R_marker @ Rx(180°)
        t_tcp = t_marker          (TCP touches marker centre)

    Because ``left_tcp`` is a fixed link 0.244 m along ``+Z`` from
    ``left_tool0``, MoveIt automatically backs ``left_tool0`` 0.244 m away
    from the marker when solving IK for ``left_tcp``.

    Args:
        marker_pose_base: Marker pose in the ``left_base`` robot frame.

    Returns:
        Target ``PoseStamped`` for ``left_tcp`` in the ``left_base`` frame.
    """
    T_base_marker = _pose_to_matrix(marker_pose_base)

    T_base_tcp = np.eye(4)
    T_base_tcp[:3, :3] = T_base_marker[:3, :3] @ _Rx180   # flip gripper Z
    T_base_tcp[:3, 3] = T_base_marker[:3, 3]               # TCP at marker centre

    return _matrix_to_pose(T_base_tcp, frame_id="left_base")


def log_transform_summary(
    marker_pose_camera: PoseStamped,
    marker_pose_base: PoseStamped,
    tcp_goal: PoseStamped,
    logger,
) -> None:
    """Emit a concise transform-chain summary to a ROS 2 logger.

    Args:
        marker_pose_camera: Marker pose in camera frame.
        marker_pose_base:   Marker pose in robot base frame.
        tcp_goal:           Computed ``left_tcp`` goal in robot base frame.
        logger:             ``rclpy`` logger from any active node.
    """
    def _pos(p):
        return f"({p.x:.4f}, {p.y:.4f}, {p.z:.4f})"

    logger.info(
        f"[transform] marker in camera  : {_pos(marker_pose_camera.pose.position)}"
    )
    logger.info(
        f"[transform] marker in base    : {_pos(marker_pose_base.pose.position)}"
    )
    logger.info(
        f"[transform] left_tcp goal     : {_pos(tcp_goal.pose.position)}"
    )

    # Decompose TCP orientation into Euler for readability.
    o = tcp_goal.pose.orientation
    rpy = Rotation.from_quat([o.x, o.y, o.z, o.w]).as_euler("xyz", degrees=True)
    logger.info(
        f"[transform] left_tcp euler XYZ (deg): "
        f"({rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f})"
    )
