from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, cast

from .config import PWM_PER_DEG, STANDING


ARM_IDS = (6, 7, 8, 17, 18, 19)
CONTROLLED_IDS = tuple(STANDING)
NOSE, LEFT_EAR, RIGHT_EAR = 0, 7, 8
LEFT_SHOULDER, RIGHT_SHOULDER = 11, 12
LEFT_ELBOW, RIGHT_ELBOW = 13, 14
LEFT_WRIST, RIGHT_WRIST = 15, 16
LEFT_HIP, RIGHT_HIP = 23, 24
LEFT_KNEE, RIGHT_KNEE = 25, 26
LEFT_ANKLE, RIGHT_ANKLE = 27, 28


@dataclass(frozen=True)
class Landmark:
    x: float
    y: float
    z: float
    visibility: float = 1.0


class VisionBodyController:
    """Convert tracked body landmarks into bounded 17-DOF body pulses."""

    def __init__(
        self,
        confidence: float = 0.60,
        lost_timeout_s: float = 0.35,
        smooth_tau_s: float = 0.12,
        max_pwm_per_s: float = 900.0,
        lift_pwm: int = 820,
        shoulder_pwm: int = 420,
        elbow_pwm: int = 260,
        head_pwm: int = 180,
        squat_deg: float = 16.0,
        leg_lift_threshold_ratio: float = 0.30,
        min_body_scale: float = 0.10,
    ) -> None:
        self.confidence = max(0.0, min(1.0, confidence))
        self.lost_timeout_s = max(0.0, lost_timeout_s)
        self.smooth_tau_s = max(0.01, smooth_tau_s)
        self.max_pwm_per_s = max(1.0, max_pwm_per_s)
        self.lift_pwm = max(0, lift_pwm)
        self.shoulder_pwm = max(0, shoulder_pwm)
        self.elbow_pwm = max(0, elbow_pwm)
        self.head_pwm = max(0, head_pwm)
        self.squat_deg = max(0.0, min(24.0, squat_deg))
        self.leg_lift_threshold_ratio = max(0.10, leg_lift_threshold_ratio)
        self.min_body_scale = max(0.05, min(0.40, min_body_scale))
        self.pose = dict(STANDING)
        self.tracked = False
        self.visible_parts: tuple[str, ...] = ()
        self.lifted_leg: Optional[str] = None
        self._last_update_s: Optional[float] = None
        self._last_seen_s: Optional[float] = None
        self._lift_candidate: Optional[str] = None
        self._lift_candidate_frames = 0

    def reset(self) -> None:
        self.pose = dict(STANDING)
        self.tracked = False
        self.visible_parts = ()
        self.lifted_leg = None
        self._last_update_s = None
        self._last_seen_s = None
        self._lift_candidate = None
        self._lift_candidate_frames = 0

    def update(self, landmarks: Optional[Sequence[object]], now_s: float, armed: bool) -> dict[int, int]:
        dt = 1.0 / 30.0 if self._last_update_s is None else max(0.005, min(0.20, now_s - self._last_update_s))
        self._last_update_s = now_s

        target = None
        detected_lifted_leg = None
        if armed and landmarks is not None:
            result = self._target_from_landmarks(landmarks)
            if result is not None:
                target, detected_lifted_leg, visible_parts = result
                self._last_seen_s = now_s
                self.tracked = True
                self.visible_parts = visible_parts
                self._update_lift_intent(detected_lifted_leg)

        if target is None:
            recently_seen = (
                armed
                and self._last_seen_s is not None
                and now_s - self._last_seen_s <= self.lost_timeout_s
            )
            self.tracked = False
            self.visible_parts = ()
            self.lifted_leg = None
            self._lift_candidate = None
            self._lift_candidate_frames = 0
            if recently_seen:
                return dict(self.pose)
            target = dict(STANDING)

        alpha = 1.0 - math.exp(-dt / self.smooth_tau_s)
        max_delta = self.max_pwm_per_s * dt
        for sid in CONTROLLED_IDS:
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

    def _target_from_landmarks(
        self,
        source: Sequence[object],
    ) -> Optional[tuple[dict[int, int], Optional[str], tuple[str, ...]]]:
        if len(source) <= RIGHT_ANKLE:
            return None

        torso_indices = (LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP)
        torso_points = [self._point(source[index]) for index in torso_indices]
        if any(point is None for point in torso_points):
            return None
        point_map = dict(zip(torso_indices, torso_points))

        shoulder_mid = self._mid(point_map[LEFT_SHOULDER], point_map[RIGHT_SHOULDER])
        hip_mid = self._mid(point_map[LEFT_HIP], point_map[RIGHT_HIP])
        up = self._normalize(self._sub(shoulder_mid, hip_mid))
        lateral = self._normalize(self._sub(point_map[RIGHT_SHOULDER], point_map[LEFT_SHOULDER]))
        if up is None or lateral is None:
            return None
        torso_height = self._length(self._sub(shoulder_mid, hip_mid))
        shoulder_width = self._length(
            self._sub(point_map[RIGHT_SHOULDER], point_map[LEFT_SHOULDER])
        )
        if torso_height < self.min_body_scale or shoulder_width < self.min_body_scale * 0.55:
            return None

        target = dict(STANDING)
        visible_parts: list[str] = []

        left_arm = self._points(source, LEFT_SHOULDER, LEFT_ELBOW, LEFT_WRIST)
        if left_arm is not None:
            left_lift, left_sweep, left_elbow = self._arm_values(*left_arm, up, lateral)
            target[18] = self._clamp_servo(18, STANDING[18] - round(left_lift * self.lift_pwm))
            target[17] = self._clamp_servo(17, STANDING[17] + round(left_sweep * self.shoulder_pwm))
            target[19] = self._clamp_servo(19, STANDING[19] - round(left_elbow * self.elbow_pwm))
            visible_parts.append("left_arm")

        right_arm = self._points(source, RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST)
        if right_arm is not None:
            right_lift, right_sweep, right_elbow = self._arm_values(*right_arm, up, lateral)
            target[7] = self._clamp_servo(7, STANDING[7] + round(right_lift * self.lift_pwm))
            target[8] = self._clamp_servo(8, STANDING[8] + round(right_sweep * self.shoulder_pwm))
            target[6] = self._clamp_servo(6, STANDING[6] + round(right_elbow * self.elbow_pwm))
            visible_parts.append("right_arm")

        head_points = self._points(source, NOSE, LEFT_EAR, RIGHT_EAR)
        if head_points is not None:
            nose, left_ear, right_ear = head_points
            ear_mid = self._mid(left_ear, right_ear)
            head_offset = self._signed_deadband(
                self._dot(self._sub(nose, ear_mid), lateral) / max(0.05, shoulder_width * 0.35),
                0.12,
            )
            target[16] = self._clamp_servo(
                16,
                STANDING[16] + round(self._clamp_unit(head_offset) * self.head_pwm),
            )
            visible_parts.append("head")

        lifted_leg = None
        legs = self._points(
            source,
            LEFT_HIP, LEFT_KNEE, LEFT_ANKLE,
            RIGHT_HIP, RIGHT_KNEE, RIGHT_ANKLE,
        )
        if legs is not None:
            left_hip, left_knee, left_ankle, right_hip, right_knee, right_ankle = legs
            left_flex = self._joint_flex(left_hip, left_knee, left_ankle)
            right_flex = self._joint_flex(right_hip, right_knee, right_ankle)
            if left_flex is not None and right_flex is not None:
                ankle_delta = self._dot(self._sub(left_ankle, right_ankle), up) / torso_height
                if ankle_delta > self.leg_lift_threshold_ratio and left_flex > 0.18:
                    lifted_leg = "left"
                elif ankle_delta < -self.leg_lift_threshold_ratio and right_flex > 0.18:
                    lifted_leg = "right"

                if lifted_leg is None:
                    squat = self._positive_deadband((left_flex + right_flex) * 0.5, 0.12)
                    hip_delta = round(self.squat_deg * 0.5 * PWM_PER_DEG * squat)
                    knee_delta = round(self.squat_deg * PWM_PER_DEG * squat)
                    ankle_pwm = round(self.squat_deg * 0.5 * PWM_PER_DEG * squat)
                    target[4] = self._clamp_servo(4, STANDING[4] - hip_delta)
                    target[3] = self._clamp_servo(3, STANDING[3] - knee_delta)
                    target[2] = self._clamp_servo(2, STANDING[2] - ankle_pwm)
                    target[21] = self._clamp_servo(21, STANDING[21] + hip_delta)
                    target[22] = self._clamp_servo(22, STANDING[22] + knee_delta)
                    target[23] = self._clamp_servo(23, STANDING[23] + ankle_pwm)
                visible_parts.append("legs")

        if not visible_parts:
            return None
        return target, lifted_leg, tuple(visible_parts)

    def _update_lift_intent(self, detected: Optional[str]) -> None:
        if detected == self._lift_candidate:
            self._lift_candidate_frames += 1
        else:
            self._lift_candidate = detected
            self._lift_candidate_frames = 1
        if self._lift_candidate_frames >= 3:
            self.lifted_leg = detected

    def _point(self, value: object) -> Optional[Landmark]:
        try:
            point = Landmark(float(value.x), float(value.y), float(value.z), float(value.visibility))
        except (AttributeError, TypeError, ValueError):
            return None
        return point if point.visibility >= self.confidence else None

    def _points(self, source: Sequence[object], *indices: int) -> Optional[tuple[Landmark, ...]]:
        points = tuple(self._point(source[index]) for index in indices)
        if any(point is None for point in points):
            return None
        return cast(tuple[Landmark, ...], points)

    def _arm_values(
        self,
        shoulder: Landmark,
        elbow: Landmark,
        wrist: Landmark,
        up: tuple[float, float, float],
        lateral: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        upper = self._normalize(self._sub(elbow, shoulder))
        forearm_from_elbow = self._normalize(self._sub(wrist, elbow))
        upper_to_shoulder = self._normalize(self._sub(shoulder, elbow))
        if upper is None or forearm_from_elbow is None or upper_to_shoulder is None:
            return 0.0, 0.0, 0.0

        down = tuple(-value for value in up)
        elevation = math.acos(self._clamp_unit(self._dot(upper, down))) / math.pi
        sweep = self._signed_deadband(self._dot(upper, lateral), 0.12)
        elbow_angle = math.acos(self._clamp_unit(self._dot(upper_to_shoulder, forearm_from_elbow)))
        elbow_flex = max(0.0, min(1.0, (math.pi - elbow_angle) / (math.pi * 0.78)))
        return elevation, sweep, elbow_flex

    def _joint_flex(
        self,
        proximal: Landmark,
        joint: Landmark,
        distal: Landmark,
    ) -> Optional[float]:
        to_proximal = self._normalize(self._sub(proximal, joint))
        to_distal = self._normalize(self._sub(distal, joint))
        if to_proximal is None or to_distal is None:
            return None
        angle = math.acos(self._clamp_unit(self._dot(to_proximal, to_distal)))
        return max(0.0, min(1.0, (math.pi - angle) / (math.pi * 0.5)))

    @staticmethod
    def _clamp_servo(sid: int, value: int) -> int:
        limits = {
            1: (1400, 1600),
            2: (1380, 1500),
            3: (1280, 1500),
            4: (1380, 1500),
            5: (1400, 1600),
            6: (1500, 1760),
            7: (500, 1320),
            8: (1050, 1890),
            16: (1320, 1680),
            17: (1080, 1920),
            18: (1630, 2450),
            19: (1240, 1500),
            20: (1400, 1600),
            21: (1500, 1620),
            22: (1500, 1720),
            23: (1500, 1620),
            24: (1400, 1600),
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
    def _length(vector: tuple[float, float, float]) -> float:
        return math.sqrt(sum(value * value for value in vector))

    @staticmethod
    def _clamp_unit(value: float) -> float:
        return max(-1.0, min(1.0, value))

    @classmethod
    def _signed_deadband(cls, value: float, threshold: float) -> float:
        value = cls._clamp_unit(value)
        magnitude = abs(value)
        if magnitude <= threshold:
            return 0.0
        scaled = (magnitude - threshold) / max(1e-6, 1.0 - threshold)
        return math.copysign(min(1.0, scaled), value)

    @staticmethod
    def _positive_deadband(value: float, threshold: float) -> float:
        value = max(0.0, min(1.0, value))
        if value <= threshold:
            return 0.0
        return min(1.0, (value - threshold) / max(1e-6, 1.0 - threshold))
