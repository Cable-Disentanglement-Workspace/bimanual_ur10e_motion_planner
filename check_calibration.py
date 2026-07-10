"""Verify hand-eye calibration and compute the measured inter-arm transform.

Runs standalone (no ROS required):
    python3 check_calibration.py

What this script does
---------------------
1. Loads both hand-eye calibration files (camera -> left_base, camera -> right_base).
2. Computes the measured transform from right_base to left_base:
       T_left_from_right = T_left_cam @ inv(T_right_cam)
3. Computes the same transform from the URDF hardcoded values for comparison.
4. Reports the translation and rotation error so you can decide whether the
   URDF values need updating.
"""

from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

CALIB_DIR = Path(__file__).parent / "calibrations"
LEFT_CALIB  = CALIB_DIR / "zed_left_arm_calib.calib"
RIGHT_CALIB = CALIB_DIR / "zed_right_arm_calib.calib"

# URDF hardcoded values from dual_arm_real_v2.urdf.xacro
URDF_LEFT_XYZ   = np.array([-0.60, -0.15, 0.0])
URDF_RIGHT_XYZ  = np.array([ 0.60, -0.15, 0.0])
URDF_LEFT_RPY   = np.array([0.0, 0.0,  1.5708])  # current: same for both arms
URDF_RIGHT_RPY  = np.array([0.0, 0.0,  1.5708])

# Hypothesis: left arm should face the opposite direction
# (clue: left gripper has yaw=pi, right gripper has yaw=0)
HYPO_LEFT_RPY   = np.array([0.0, 0.0, -1.5708])


# ── helpers ───────────────────────────────────────────────────────────────────

def load_calib(path: Path) -> np.ndarray:
    """Return 4x4 T_base_camera from an easy_handeye2 .calib file."""
    data = yaml.safe_load(path.read_text())
    t = data["transform"]["translation"]
    r = data["transform"]["rotation"]
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([r["x"], r["y"], r["z"], r["w"]]).as_matrix()
    T[:3, 3]  = [t["x"], t["y"], t["z"]]
    return T


def make_transform(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    """Build a 4x4 homogeneous matrix from xyz translation and rpy (rad)."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    T[:3, 3]  = xyz
    return T


def rotation_error_deg(R: np.ndarray) -> float:
    """Angle (degrees) of the rotation matrix R."""
    return np.degrees(Rotation.from_matrix(R).magnitude())


def print_transform(label: str, T: np.ndarray) -> None:
    xyz = T[:3, 3]
    rpy = np.degrees(Rotation.from_matrix(T[:3, :3]).as_euler("xyz"))
    quat = Rotation.from_matrix(T[:3, :3]).as_quat()
    print(f"\n  {label}")
    print(f"    translation (m) : x={xyz[0]:+.4f}  y={xyz[1]:+.4f}  z={xyz[2]:+.4f}")
    print(f"    rotation (deg)  : r={rpy[0]:+.2f}  p={rpy[1]:+.2f}  y={rpy[2]:+.2f}")
    print(f"    quaternion      : x={quat[0]:+.4f}  y={quat[1]:+.4f}  z={quat[2]:+.4f}  w={quat[3]:+.4f}")


# ── main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    print("=" * 60)
    print("  Bimanual UR10e — calibration verification")
    print("=" * 60)

    # 1. Load hand-eye calibrations
    T_left_cam  = load_calib(LEFT_CALIB)
    T_right_cam = load_calib(RIGHT_CALIB)
    print(f"\n[OK] Loaded {LEFT_CALIB.name}")
    print(f"[OK] Loaded {RIGHT_CALIB.name}")

    print_transform("T_left_base  <- camera (hand-eye left)", T_left_cam)
    print_transform("T_right_base <- camera (hand-eye right)", T_right_cam)

    # 2. Measured inter-arm transform (camera as common reference)
    #    T_left_from_right: maps a point in right_base -> left_base
    T_left_from_right_measured = T_left_cam @ np.linalg.inv(T_right_cam)

    # 3. URDF inter-arm transform (current)
    T_table_left_current  = make_transform(URDF_LEFT_XYZ,  URDF_LEFT_RPY)
    T_table_right_current = make_transform(URDF_RIGHT_XYZ, URDF_RIGHT_RPY)
    T_left_from_right_urdf = np.linalg.inv(T_table_left_current) @ T_table_right_current

    # 4. Hypothesis: left arm rpy = -1.5708 (arms face each other)
    T_table_left_hypo  = make_transform(URDF_LEFT_XYZ,  HYPO_LEFT_RPY)
    T_left_from_right_hypo = np.linalg.inv(T_table_left_hypo) @ T_table_right_current

    # 5. Report
    print("\n" + "=" * 60)
    print("  Inter-arm transform: right_base -> left_base")
    print("=" * 60)
    print_transform("Measured (from hand-eye calibration)",            T_left_from_right_measured)
    print_transform("URDF current  (both rpy=+1.5708)",               T_left_from_right_urdf)
    print_transform("URDF hypothesis (left rpy=-1.5708, right=+1.5708)", T_left_from_right_hypo)

    def err(T_ref, T_test):
        T_e = np.linalg.inv(T_ref) @ T_test
        return np.linalg.norm(T_e[:3, 3]) * 1000, rotation_error_deg(T_e[:3, :3])

    t_err_cur,  r_err_cur  = err(T_left_from_right_measured, T_left_from_right_urdf)
    t_err_hypo, r_err_hypo = err(T_left_from_right_measured, T_left_from_right_hypo)

    print("\n" + "=" * 60)
    print("  Error vs measured")
    print("=" * 60)
    print(f"    Current URDF  — translation: {t_err_cur:.1f} mm   rotation: {r_err_cur:.2f} deg")
    print(f"    Hypothesis    — translation: {t_err_hypo:.1f} mm   rotation: {r_err_hypo:.2f} deg")

    if t_err_hypo < t_err_cur:
        print("\n  >> Hypothesis (left rpy=-1.5708) is a BETTER match.")
        print("     Physical check needed: verify which direction left arm faces on the table.")
        if t_err_hypo < 5.0 and r_err_hypo < 0.5:
            print("     Distance error after orientation fix: GOOD (< 5 mm).")
        else:
            print(f"     Remaining distance error: {t_err_hypo:.1f} mm — update xyz values too.")
            m_xyz = T_left_from_right_measured[:3, 3]
            print(f"     Measured right_base in left_base: x={m_xyz[0]:+.4f} y={m_xyz[1]:+.4f} z={m_xyz[2]:+.4f}")
    else:
        print("\n  >> Current URDF orientation is the better match.")
        if t_err_cur >= 5.0 or r_err_cur >= 0.5:
            m_xyz = T_left_from_right_measured[:3, 3]
            print(f"     Update xyz: x={m_xyz[0]:+.4f} y={m_xyz[1]:+.4f} z={m_xyz[2]:+.4f}")


if __name__ == "__main__":
    main()
