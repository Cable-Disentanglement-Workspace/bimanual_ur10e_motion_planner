"""
Quaternion ↔ Euler Angles Codec
================================
Reads a quaternion from a JSON file and converts it to Euler angles
for all 12 valid rotation-sequence combinations (intrinsic).
Results are saved to a JSON file in radians.

Input JSON format (either flat or nested is accepted):
    { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
    or
    { "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 } }

Usage:
    python quat_euler_codec.py --input input.json --output output.json
    python quat_euler_codec.py --input input.json   # writes to euler_output.json
    python quat_euler_codec.py --demo               # generate sample input + run
"""

import argparse
import json
import sys
from pathlib import Path
from scipy.spatial.transform import Rotation


# ---------------------------------------------------------------------------
# All 12 valid rotation sequences (Tait-Bryan + proper Euler), intrinsic
# ---------------------------------------------------------------------------
EULER_SEQUENCES = [
    "xyz", "xzy", "yxz", "yzx", "zxy", "zyx",   # Tait-Bryan
    "xyx", "xzx", "yxy", "yzy", "zxz", "zyz",   # Proper Euler
]


# ---------------------------------------------------------------------------
# Core helpers
# ---------------------------------------------------------------------------

def load_quaternion(filepath: str) -> tuple:
    """Load a quaternion from a JSON file.

    Accepts two JSON layouts:
        - Flat:  ``{"x": ..., "y": ..., "z": ..., "w": ...}``
        - Nested: ``{"orientation": {"x": ..., "y": ..., "z": ..., "w": ...}}``

    Args:
        filepath: Path to the input JSON file.

    Returns:
        Tuple of ``(x, y, z, w)`` as floats.

    Raises:
        FileNotFoundError: If *filepath* does not exist.
        KeyError: If required quaternion keys are missing.
        ValueError: If the quaternion norm is near zero.
    """
    path = Path(filepath)
    if not path.exists():
        raise FileNotFoundError(f"Input file not found: {filepath}")

    with path.open("r") as fh:
        data = json.load(fh)

    # Support flat or nested orientation key
    if "orientation" in data:
        data = data["orientation"]

    try:
        x, y, z, w = float(data["x"]), float(data["y"]), float(data["z"]), float(data["w"])
    except KeyError as exc:
        raise KeyError(f"Missing quaternion component {exc} in JSON.") from exc

    norm = (x**2 + y**2 + z**2 + w**2) ** 0.5
    if norm < 1e-9:
        raise ValueError("Quaternion has near-zero norm — cannot normalise.")

    return x, y, z, w


def quaternion_to_all_euler(x: float, y: float, z: float, w: float) -> dict:
    """Convert a quaternion to Euler angles for every rotation sequence.

    Uses ``scipy.spatial.transform.Rotation`` with the scalar-last
    convention ``[x, y, z, w]``, matching ROS / most robotics toolchains.
    All output angles are in **radians**.

    For proper-Euler sequences (e.g. ``"xyx"``) the two identical axes share
    the same letter, so angles are stored positionally as
    ``angle_1 / angle_2 / angle_3`` rather than by axis name to avoid key
    collisions.

    Args:
        x: Quaternion x component.
        y: Quaternion y component.
        z: Quaternion z component.
        w: Quaternion w component (scalar).

    Returns:
        A dict keyed by sequence string (e.g. ``"xyz"``), each value being a
        dict with keys ``axes``, ``angle_1``, ``angle_2``, ``angle_3``
        (all angles in radians). Example::

            {
                "xyz": {
                    "axes": "xyz",
                    "angle_1": 0.1,   # rotation about x
                    "angle_2": -0.2,  # rotation about y
                    "angle_3": 0.05   # rotation about z
                },
                ...
            }
    """
    rot = Rotation.from_quat([x, y, z, w])  # scalar-last [x, y, z, w]
    results = {}

    for seq in EULER_SEQUENCES:
        angles = rot.as_euler(seq, degrees=False)  # ndarray shape (3,)
        results[seq] = {
            "axes": seq,
            "angle_1": float(angles[0]),
            "angle_2": float(angles[1]),
            "angle_3": float(angles[2]),
        }

    return results


def save_results(output_path: str, quaternion: tuple, euler_map: dict) -> None:
    """Save the input quaternion and all Euler angle results to a JSON file.

    Args:
        output_path: Destination file path (created or overwritten).
        quaternion: The source quaternion as ``(x, y, z, w)``.
        euler_map: Output of :func:`quaternion_to_all_euler`.
    """
    x, y, z, w = quaternion
    payload = {
        "input_quaternion": {"x": x, "y": y, "z": z, "w": w},
        "euler_angles_radians": euler_map,
    }
    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w") as fh:
        json.dump(payload, fh, indent=2)
    print(f"[✓] Results written to: {out.resolve()}")


def print_summary(euler_map: dict) -> None:
    """Print a human-readable table of all Euler conversions to stdout.

    Args:
        euler_map: Output of :func:`quaternion_to_all_euler`.
    """
    header = "{:<6}  {:>14}  {:>14}  {:>14}".format(
        "Seq", "Angle-1 (rad)", "Angle-2 (rad)", "Angle-3 (rad)"
    )
    print("\n" + header)
    print("-" * 56)
    for seq, entry in euler_map.items():
        a1, a2, a3 = entry["angle_1"], entry["angle_2"], entry["angle_3"]
        print(f"{seq:<6}  {a1:>14.6f}  {a2:>14.6f}  {a3:>14.6f}")


# ---------------------------------------------------------------------------
# Demo helpers
# ---------------------------------------------------------------------------

def create_demo_input(path: str = "demo_input.json") -> None:
    """Write a sample quaternion JSON file for quick testing.

    The sample represents a rotation of roughly 45° about a diagonal axis.

    Args:
        path: File path to write the demo input to.
    """
    sample = {"x": 0.1830, "y": 0.3696, "z": 0.0747, "w": 0.9083}
    with open(path, "w") as fh:
        json.dump(sample, fh, indent=2)
    print(f"[demo] Sample input written to: {path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    """Build the command-line argument parser.

    Returns:
        Configured :class:`argparse.ArgumentParser` instance.
    """
    parser = argparse.ArgumentParser(
        description="Convert a quaternion (JSON) to Euler angles for all 12 sequences.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--input", "-i", metavar="FILE",
        help="Input JSON file containing the quaternion.",
    )
    parser.add_argument(
        "--output", "-o", metavar="FILE", default="euler_output.json",
        help="Output JSON file (default: euler_output.json).",
    )
    parser.add_argument(
        "--demo", action="store_true",
        help="Generate a demo input file and run conversion.",
    )
    return parser


def main() -> None:
    """Entry point: parse arguments, load quaternion, convert, and save."""
    parser = build_parser()
    args = parser.parse_args()

    if args.demo:
        demo_in = "demo_input.json"
        create_demo_input(demo_in)
        args.input = demo_in

    if not args.input:
        parser.print_help()
        sys.exit(1)

    # 1. Load
    print(f"[→] Loading quaternion from: {args.input}")
    quat = load_quaternion(args.input)
    x, y, z, w = quat
    print(f"    q = (x={x:.6f}, y={y:.6f}, z={z:.6f}, w={w:.6f})")

    # 2. Convert
    euler_map = quaternion_to_all_euler(x, y, z, w)

    # 3. Print summary
    print_summary(euler_map)

    # 4. Save
    save_results(args.output, quat, euler_map)


if __name__ == "__main__":
    main()