"""
Simplified leg inverse kinematics for the walking engine.

The model separates frontal-plane hip abduction from sagittal-plane
hip/knee/ankle pitch. It is intentionally lightweight because it runs every
control frame on the Raspberry Pi.
"""

from __future__ import annotations

import math


def leg_ik(hip: tuple[float, float, float], foot: tuple[float, float, float], L1: float, L2: float) -> dict[str, float]:
    """
    Compute one-leg joint angles from hip and foot positions.

    Args:
        hip: (x, y, z) hip joint position in mm.
        foot: (x, y, z) ankle/foot target position in mm.
        L1: upper leg length in mm.
        L2: lower leg length in mm.

    Returns:
        hip_abduct: hip roll/abduction angle in degrees.
        hip_pitch: hip pitch angle in degrees.
        knee: knee bend angle in degrees, where 0 means straight.
        ankle_pitch: calibrated ankle pitch compensation in degrees. The current
            model solves the ankle/foot-point position; it does not solve a full
            6D foot orientation constraint.
    """
    dx = foot[0] - hip[0]
    dy = foot[1] - hip[1]
    dz = foot[2] - hip[2]

    leg_len_frontal = math.sqrt(dy**2 + dz**2)
    hip_abduct = math.degrees(math.atan2(dy, -dz))

    L = math.sqrt(dx**2 + leg_len_frontal**2)
    L = max(min(L, L1 + L2 - 0.5), abs(L1 - L2) + 0.5)

    cos_k = (L1**2 + L2**2 - L**2) / (2 * L1 * L2)
    cos_k = max(-1.0, min(1.0, cos_k))
    gamma = math.degrees(math.acos(cos_k))
    knee = 180.0 - gamma

    cos_h = (L1**2 + L**2 - L2**2) / (2 * L1 * L)
    cos_h = max(-1.0, min(1.0, cos_h))
    alpha = math.degrees(math.acos(cos_h))
    beta = math.degrees(math.atan2(dx, leg_len_frontal))
    hip_pitch = beta + alpha

    # This compensation follows the existing servo calibration convention used
    # by walking_engine.STAND_ANG and DIR. A full foot-orientation controller
    # would derive the ankle sign from the mounted foot frame instead.
    ankle_pitch = knee - hip_pitch

    return {
        "hip_abduct": hip_abduct,
        "hip_pitch": hip_pitch,
        "knee": knee,
        "ankle_pitch": ankle_pitch,
    }
