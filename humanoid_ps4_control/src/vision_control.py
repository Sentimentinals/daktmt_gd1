from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

from .config import STANDING


ARM_IDS = (6, 7, 8, 17, 18, 19)
LEFT_SHOULDER, RIGHT_SHOULDER = 11, 12
LEFT_ELBOW, RIGHT_ELBOW = 13, 14
LEFT_WRIST, RIGHT_WRIST = 15, 16
LEFT_HIP, RIGHT_HIP = 23, 24


@dataclass(frozen=True)
class Landmark:
    x: float
    y: float
    z: float
    visibility: float = 1.0


class VisionArmController:
    """Convert MediaPipe world landmarks into bounded arm servo pulses."""

    def __init__(
        self,
        confidence: float = 0.60,
        lost_timeout_s: float = 0.35,
        smooth_tau_s: float = 0.12,
        max_pwm_per_s: float = 900.0,
        lift_pwm: int = 820,
        shoulder_pwm: int = 420,
        elbow_pwm: int = 260,
    ) -> None:
        self.confidence = max(0.0, min(1.0, confidence))
        self.lost_timeout_s = max(0.0, lost_timeout_s)
        self.smooth_tau_s = max(0.01, smooth_tau_s)
        self.max_pwm_per_s = max(1.0, max_pwm_per_s)
        self.lift_pwm = max(0, lift_pwm)
        self.shoulder_pwm = max(0, shoulder_pwm)
        self.elbow_pwm = max(0, elbow_pwm)
        self.pose = dict(STANDING)
        self.tracked = False
        self._last_update_s: Optional[float] = None
        self._last_seen_s: Optional[float] = None

    def reset(self) -> None:
        self.pose = dict(STANDING)
        self.tracked = False
        self._last_update_s = None
        self._last_seen_s = None

    def update(self, landmarks: Optional[Sequence[object]], now_s: float, armed: bool) -> dict[int, int]:
        dt = 1.0 / 30.0 if self._last_update_s is None else max(0.005, min(0.20, now_s - self._last_update_s))
        self._last_update_s = now_s

        target = None
        if armed and landmarks is not None:
            target = self._target_from_landmarks(landmarks)
            if target is not None:
                self._last_seen_s = now_s
                self.tracked = True

        if target is None:
            recently_seen = (
                armed
                and self._last_seen_s is not None
                and now_s - self._last_seen_s <= self.lost_timeout_s
            )
            if recently_seen:
                return dict(self.pose)
            self.tracked = False
            target = dict(STANDING)

        alpha = 1.0 - math.exp(-dt / self.smooth_tau_s)
        max_delta = self.max_pwm_per_s * dt
        for sid in ARM_IDS:
            current = float(self.pose[sid])
            desired = float(target[sid])
            delta = (desired - current) * alpha
            delta = max(-max_delta, min(max_delta, delta))
            if abs(desired - current) <= 1.0:
                next_value = round(desired)
            else:
                next_value = round(current + delta)
                if next_value == round(current):
                    next_value += 1 if desired > current else -1
            self.pose[sid] = self._clamp_servo(sid, next_value)
        return dict(self.pose)

    def _target_from_landmarks(self, source: Sequence[object]) -> Optional[dict[int, int]]:
        required = (
            LEFT_SHOULDER,
            RIGHT_SHOULDER,
            LEFT_ELBOW,
            RIGHT_ELBOW,
            LEFT_WRIST,
            RIGHT_WRIST,
            LEFT_HIP,
            RIGHT_HIP,
        )
        if len(source) <= max(required):
            return None

        points = [self._point(source[index]) for index in required]
        if any(point is None for point in points):
            return None
        point_map = dict(zip(required, points))

        shoulder_mid = self._mid(point_map[LEFT_SHOULDER], point_map[RIGHT_SHOULDER])
        hip_mid = self._mid(point_map[LEFT_HIP], point_map[RIGHT_HIP])
        up = self._normalize(self._sub(shoulder_mid, hip_mid))
        lateral = self._normalize(self._sub(point_map[RIGHT_SHOULDER], point_map[LEFT_SHOULDER]))
        if up is None or lateral is None:
            return None
        forward = self._normalize(self._cross(lateral, up))
        if forward is None:
            return None

        target = dict(STANDING)
        left = self._arm_values(
            point_map[LEFT_SHOULDER], point_map[LEFT_ELBOW], point_map[LEFT_WRIST], up, forward
        )
        right = self._arm_values(
            point_map[RIGHT_SHOULDER], point_map[RIGHT_ELBOW], point_map[RIGHT_WRIST], up, forward
        )
        if left is None or right is None:
            return None

        left_lift, left_swing, left_elbow = left
        right_lift, right_swing, right_elbow = right
        target[7] = self._clamp_servo(7, STANDING[7] + round(right_lift * self.lift_pwm))
        target[8] = self._clamp_servo(8, STANDING[8] + round(right_swing * self.shoulder_pwm))
        target[6] = self._clamp_servo(6, STANDING[6] + round(right_elbow * self.elbow_pwm))
        target[18] = self._clamp_servo(18, STANDING[18] - round(left_lift * self.lift_pwm))
        target[17] = self._clamp_servo(17, STANDING[17] + round(left_swing * self.shoulder_pwm))
        target[19] = self._clamp_servo(19, STANDING[19] - round(left_elbow * self.elbow_pwm))
        return target

    def _point(self, value: object) -> Optional[Landmark]:
        try:
            point = Landmark(float(value.x), float(value.y), float(value.z), float(value.visibility))
        except (AttributeError, TypeError, ValueError):
            return None
        return point if point.visibility >= self.confidence else None

    def _arm_values(
        self,
        shoulder: Landmark,
        elbow: Landmark,
        wrist: Landmark,
        up: tuple[float, float, float],
        forward: tuple[float, float, float],
    ) -> Optional[tuple[float, float, float]]:
        upper = self._normalize(self._sub(elbow, shoulder))
        forearm_from_elbow = self._normalize(self._sub(wrist, elbow))
        upper_to_shoulder = self._normalize(self._sub(shoulder, elbow))
        if upper is None or forearm_from_elbow is None or upper_to_shoulder is None:
            return None

        down = tuple(-value for value in up)
        elevation = math.acos(self._clamp_unit(self._dot(upper, down))) / math.pi
        swing = self._clamp_unit(self._dot(upper, forward))
        elbow_angle = math.acos(self._clamp_unit(self._dot(upper_to_shoulder, forearm_from_elbow)))
        elbow_flex = max(0.0, min(1.0, (math.pi - elbow_angle) / (math.pi * 0.78)))
        return elevation, swing, elbow_flex

    @staticmethod
    def _clamp_servo(sid: int, value: int) -> int:
        limits = {
            6: (1500, 1760),
            7: (500, 1320),
            8: (1050, 1890),
            17: (1080, 1920),
            18: (1630, 2450),
            19: (1240, 1500),
        }
        low, high = limits.get(sid, (500, 2500))
        return max(low, min(high, int(value)))

    @staticmethod
    def _sub(a: Landmark, b: Landmark) -> tuple[float, float, float]:
        return a.x - b.x, a.y - b.y, a.z - b.z

    @staticmethod
    def _mid(a: Landmark, b: Landmark) -> Landmark:
        return Landmark((a.x + b.x) * 0.5, (a.y + b.y) * 0.5, (a.z + b.z) * 0.5)

    @staticmethod
    def _normalize(vector: tuple[float, float, float]) -> Optional[tuple[float, float, float]]:
        norm = math.sqrt(sum(value * value for value in vector))
        if norm < 1e-6:
            return None
        return tuple(value / norm for value in vector)

    @staticmethod
    def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
        return sum(a[index] * b[index] for index in range(3))

    @staticmethod
    def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
        return (
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        )

    @staticmethod
    def _clamp_unit(value: float) -> float:
        return max(-1.0, min(1.0, value))
