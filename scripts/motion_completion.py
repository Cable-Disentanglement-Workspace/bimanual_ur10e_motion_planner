"""Explicit completion acknowledgement for blocking URScript movements."""

import socket
import time


COMPLETION_TOKEN = b"URSCRIPT_MOTION_DONE"
BASE_MOVEIT_VELOCITY_SCALE = 0.02
BASE_MOVEIT_ACCELERATION_SCALE = 0.02
BASE_URSCRIPT_VELOCITY = 0.1
BASE_URSCRIPT_ACCELERATION = 0.1
MIN_MOTION_MULTIPLIER = 0.25
MAX_MOTION_MULTIPLIER = 2.0
MAX_MOVEIT_SCALING = 1.0
MAX_URSCRIPT_VELOCITY = 2.0
MAX_URSCRIPT_ACCELERATION = 5.0


def validated_motion_values(
    moveit_velocity: float,
    moveit_acceleration: float,
    urscript_velocity: float,
    urscript_acceleration: float,
):
    """Validate and return explicit MoveIt and URScript motion values."""
    values = {
        "moveit_velocity": float(moveit_velocity),
        "moveit_acceleration": float(moveit_acceleration),
        "urscript_velocity": float(urscript_velocity),
        "urscript_acceleration": float(urscript_acceleration),
    }
    limits = {
        "moveit_velocity": MAX_MOVEIT_SCALING,
        "moveit_acceleration": MAX_MOVEIT_SCALING,
        "urscript_velocity": MAX_URSCRIPT_VELOCITY,
        "urscript_acceleration": MAX_URSCRIPT_ACCELERATION,
    }
    for name, value in values.items():
        if not 0.0 < value <= limits[name]:
            raise ValueError(
                f"{name} must be greater than 0 and at most {limits[name]}"
            )
    return values


def scaled_motion_values(speed_multiplier: float, acceleration_multiplier: float):
    """Return linked dynamics for backward-compatible multiplier arguments."""
    speed_multiplier = float(speed_multiplier)
    acceleration_multiplier = float(acceleration_multiplier)
    if not MIN_MOTION_MULTIPLIER <= speed_multiplier <= MAX_MOTION_MULTIPLIER:
        raise ValueError(
            f"speed multiplier must be between {MIN_MOTION_MULTIPLIER} and "
            f"{MAX_MOTION_MULTIPLIER}"
        )
    if not MIN_MOTION_MULTIPLIER <= acceleration_multiplier <= MAX_MOTION_MULTIPLIER:
        raise ValueError(
            f"acceleration multiplier must be between {MIN_MOTION_MULTIPLIER} "
            f"and {MAX_MOTION_MULTIPLIER}"
        )
    return {
        "moveit_velocity": BASE_MOVEIT_VELOCITY_SCALE * speed_multiplier,
        "moveit_acceleration": (
            BASE_MOVEIT_ACCELERATION_SCALE * acceleration_multiplier
        ),
        "urscript_velocity": BASE_URSCRIPT_VELOCITY * speed_multiplier,
        "urscript_acceleration": (
            BASE_URSCRIPT_ACCELERATION * acceleration_multiplier
        ),
    }


def local_ip_for_peer(peer_ip: str) -> str:
    """Return the local interface address used to reach a robot."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((str(peer_ip), 30002))
        return str(probe.getsockname()[0])
    finally:
        probe.close()


class URScriptCompletionServer:
    """Listen for a token sent by URScript after its blocking movej returns."""

    def __init__(self, bind_ip: str, port: int = 0):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((str(bind_ip), int(port)))
        self._socket.listen(1)
        self.host, self.port = self._socket.getsockname()

    def close(self):
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc, _traceback):
        self.close()

    def wait(self, timeout: float, connect_timeout: float = 10.0) -> bool:
        """Wait for one client and the exact completion token."""
        self._socket.settimeout(max(0.01, float(connect_timeout)))
        try:
            connection, _address = self._socket.accept()
        except (socket.timeout, OSError):
            return False

        deadline = time.monotonic() + float(timeout)
        received = bytearray()
        with connection:
            while time.monotonic() < deadline:
                connection.settimeout(max(0.01, deadline - time.monotonic()))
                try:
                    chunk = connection.recv(1024)
                except socket.timeout:
                    return False
                if not chunk:
                    break
                received.extend(chunk)
                if COMPLETION_TOKEN in received:
                    return True
        return False
