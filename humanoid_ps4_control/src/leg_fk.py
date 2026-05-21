from __future__ import annotations

import math

import numpy as np


def rot_x(angle_deg: float) -> np.ndarray:
    """Homogeneous rotation around X axis in degrees."""
    c = math.cos(math.radians(angle_deg))
    s = math.sin(math.radians(angle_deg))
    return np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, c, -s, 0.0],
            [0.0, s, c, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def rot_y(angle_deg: float) -> np.ndarray:
    """Homogeneous rotation around Y axis in degrees."""
    c = math.cos(math.radians(angle_deg))
    s = math.sin(math.radians(angle_deg))
    return np.array(
        [
            [c, 0.0, s, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [-s, 0.0, c, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def trans_z(d: float) -> np.ndarray:
    """Homogeneous translation along Z axis."""
    return np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, d],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def leg_fk(angles: dict[str, float], L1: float, L2: float) -> np.ndarray:
    """
    Forward kinematics for one simplified 4-joint leg.

    Input angles are in degrees:
      hip_abduct, hip_pitch, knee, ankle_pitch.

    Returns the ankle/foot-point position [x, y, z] relative to the hip joint
    in mm. Foot orientation is not part of this simplified endpoint model.
    """
    hip_abduct = angles.get("hip_abduct", 0.0)
    hip_pitch = angles.get("hip_pitch", 0.0)
    knee = angles.get("knee", 0.0)
    ankle_pitch = angles.get("ankle_pitch", 0.0)

    T01 = rot_x(hip_abduct)
    T12 = rot_y(-hip_pitch)
    T23 = trans_z(-L1) @ rot_y(knee)
    T34 = trans_z(-L2) @ rot_y(ankle_pitch)

    T_foot = T01 @ T12 @ T23 @ T34
    foot_pos = T_foot @ np.array([0.0, 0.0, 0.0, 1.0])
    return foot_pos[:3]


if __name__ == "__main__":
    L1 = 80.0
    L2 = 75.0

    standing = {
        "hip_abduct": 0.0,
        "hip_pitch": 0.0,
        "knee": 0.0,
        "ankle_pitch": 0.0,
    }
    pos = leg_fk(standing, L1, L2)
    print("Test 1 - standing")
    print(f"angles: {standing}")
    print(f"foot: X={pos[0]:.1f}, Y={pos[1]:.1f}, Z={pos[2]:.1f}")

    lifted = {
        "hip_abduct": 0.0,
        "hip_pitch": 45.0,
        "knee": 90.0,
        "ankle_pitch": -45.0,
    }
    pos = leg_fk(lifted, L1, L2)
    print("\nTest 2 - bent knee")
    print(f"angles: {lifted}")
    print(f"foot: X={pos[0]:.1f}, Y={pos[1]:.1f}, Z={pos[2]:.1f}")
