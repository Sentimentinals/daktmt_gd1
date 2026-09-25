from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


def _box_center(box: tuple[int, int, int, int]) -> tuple[float, float]:
    x1, y1, x2, y2 = box
    return (0.5 * (x1 + x2), 0.5 * (y1 + y2))


def _box_iou(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    intersection = max(0, min(ax2, bx2) - max(ax1, bx1)) * max(
        0, min(ay2, by2) - max(ay1, by1)
    )
    area_a = max(1, ax2 - ax1) * max(1, ay2 - ay1)
    area_b = max(1, bx2 - bx1) * max(1, by2 - by1)
    return intersection / max(1, area_a + area_b - intersection)


@dataclass(frozen=True)
class PersonDetection:
    box: tuple[int, int, int, int]
    confidence: float
    frame_width: int
    frame_height: int
    track_id: int = 0

    @property
    def center_x_ratio(self) -> float:
        x1, _, x2, _ = self.box
        return ((x1 + x2) * 0.5) / max(1, self.frame_width)

    @property
    def height_ratio(self) -> float:
        _, y1, _, y2 = self.box
        return max(0.0, y2 - y1) / max(1, self.frame_height)


@dataclass(frozen=True)
class PersonFrame:
    people: tuple[PersonDetection, ...] = ()
    captured_at: float = 0.0

    @property
    def single_person(self) -> Optional[PersonDetection]:
        return self.people[0] if len(self.people) == 1 else None

    def person_by_id(self, track_id: int | None) -> Optional[PersonDetection]:
        if track_id is None:
            return None
        return next((person for person in self.people if person.track_id == track_id), None)


@dataclass
class _PersonTrack:
    box: tuple[int, int, int, int]
    appearance: object
    velocity_x: float = 0.0
    velocity_y: float = 0.0
    missed: int = 0

    @property
    def predicted_box(self) -> tuple[int, int, int, int]:
        dx = round(self.velocity_x)
        dy = round(self.velocity_y)
        x1, y1, x2, y2 = self.box
        return x1 + dx, y1 + dy, x2 + dx, y2 + dy


class PersonDetector:
    PERSON_CLASS_ID = 15

    def __init__(
        self,
        prototxt_path: str,
        model_path: str,
        confidence: float = 0.55,
        detect_every_frames: int = 3,
    ) -> None:
        import cv2

        prototxt = Path(prototxt_path)
        model = Path(model_path)
        if not prototxt.is_file() or not model.is_file():
            raise FileNotFoundError(
                f"Person detector model missing: {prototxt} or {model}"
            )
        self._cv2 = cv2
        self._net = cv2.dnn.readNet(str(model), str(prototxt))
        self.confidence = max(0.0, min(1.0, confidence))
        self.detect_every_frames = max(1, detect_every_frames)
        self._frame_count = 0
        self._last = PersonFrame()
        self._tracks: dict[int, _PersonTrack] = {}
        self._next_track_id = 1
        self._max_track_misses = 5

    def _appearance(self, frame, box: tuple[int, int, int, int]):
        x1, y1, x2, y2 = box
        body_top = y1 + round((y2 - y1) * 0.18)
        body_bottom = y1 + round((y2 - y1) * 0.82)
        crop = frame[body_top:body_bottom, x1:x2]
        if crop.size == 0:
            return None
        hsv = self._cv2.cvtColor(crop, self._cv2.COLOR_RGB2HSV)
        histogram = self._cv2.calcHist([hsv], [0, 1], None, [8, 8], [0, 180, 0, 256])
        self._cv2.normalize(histogram, histogram)
        return histogram

    def _assign_track_ids(
        self,
        detections: list[PersonDetection],
        appearances: list[object],
    ) -> list[PersonDetection]:
        candidates = []
        for track_id, track in self._tracks.items():
            predicted = track.predicted_box
            predicted_center = _box_center(predicted)
            predicted_area = max(1, predicted[2] - predicted[0]) * max(1, predicted[3] - predicted[1])
            for index, detection in enumerate(detections):
                center = _box_center(detection.box)
                diagonal = max(1.0, (detection.frame_width**2 + detection.frame_height**2) ** 0.5)
                center_distance = (
                    (center[0] - predicted_center[0]) ** 2
                    + (center[1] - predicted_center[1]) ** 2
                ) ** 0.5 / diagonal
                overlap = _box_iou(predicted, detection.box)
                area = max(1, detection.box[2] - detection.box[0]) * max(
                    1, detection.box[3] - detection.box[1]
                )
                size_score = min(area, predicted_area) / max(area, predicted_area)
                appearance_score = 0.5
                if track.appearance is not None and appearances[index] is not None:
                    correlation = self._cv2.compareHist(
                        track.appearance,
                        appearances[index],
                        self._cv2.HISTCMP_CORREL,
                    )
                    appearance_score = max(0.0, min(1.0, 0.5 * (correlation + 1.0)))
                position_score = max(0.0, 1.0 - center_distance / 0.30)
                score = 0.38 * overlap + 0.30 * position_score + 0.22 * appearance_score + 0.10 * size_score
                if overlap >= 0.04 or center_distance <= 0.18:
                    candidates.append((score, track_id, index))

        matched_tracks: set[int] = set()
        matched_detections: set[int] = set()
        assignments: dict[int, int] = {}
        for score, track_id, index in sorted(candidates, reverse=True):
            if score < 0.30 or track_id in matched_tracks or index in matched_detections:
                continue
            matched_tracks.add(track_id)
            matched_detections.add(index)
            assignments[index] = track_id

        for track_id in list(self._tracks):
            track = self._tracks[track_id]
            if track_id not in matched_tracks:
                track.missed += 1
                if track.missed > self._max_track_misses:
                    del self._tracks[track_id]

        tracked_people = []
        for index, detection in enumerate(detections):
            track_id = assignments.get(index)
            if track_id is None:
                track_id = self._next_track_id
                self._next_track_id += 1
                self._tracks[track_id] = _PersonTrack(
                    box=detection.box,
                    appearance=appearances[index],
                )
            else:
                track = self._tracks[track_id]
                old_center = _box_center(track.box)
                new_center = _box_center(detection.box)
                track.velocity_x = 0.55 * track.velocity_x + 0.45 * (new_center[0] - old_center[0])
                track.velocity_y = 0.55 * track.velocity_y + 0.45 * (new_center[1] - old_center[1])
                track.box = detection.box
                track.missed = 0
                if appearances[index] is not None:
                    if track.appearance is None:
                        track.appearance = appearances[index]
                    else:
                        track.appearance = 0.75 * track.appearance + 0.25 * appearances[index]
                        self._cv2.normalize(track.appearance, track.appearance)
            tracked_people.append(
                PersonDetection(
                    box=detection.box,
                    confidence=detection.confidence,
                    frame_width=detection.frame_width,
                    frame_height=detection.frame_height,
                    track_id=track_id,
                )
            )
        return tracked_people

    def detect(self, frame) -> PersonFrame:
        self._frame_count += 1
        if self._frame_count % self.detect_every_frames:
            return self._last

        height, width = frame.shape[:2]
        blob = self._cv2.dnn.blobFromImage(
            frame,
            0.007843,
            (300, 300),
            127.5,
        )
        self._net.setInput(blob)
        raw = self._net.forward()
        people = []
        appearances = []
        for index in range(raw.shape[2]):
            confidence = float(raw[0, 0, index, 2])
            class_id = int(raw[0, 0, index, 1])
            if confidence < self.confidence or class_id != self.PERSON_CLASS_ID:
                continue
            values = raw[0, 0, index, 3:7]
            x1 = max(0, min(width - 1, int(values[0] * width)))
            y1 = max(0, min(height - 1, int(values[1] * height)))
            x2 = max(0, min(width - 1, int(values[2] * width)))
            y2 = max(0, min(height - 1, int(values[3] * height)))
            if x2 <= x1 or y2 <= y1:
                continue
            detection = PersonDetection(
                box=(x1, y1, x2, y2),
                confidence=confidence,
                frame_width=width,
                frame_height=height,
            )
            people.append(detection)
            appearances.append(self._appearance(frame, detection.box))

        people = self._assign_track_ids(people, appearances)
        people.sort(key=lambda person: person.confidence, reverse=True)
        self._last = PersonFrame(tuple(people), time.monotonic())
        return self._last


class PersonFollowController:
    def __init__(
        self,
        turn_deadband: float,
        stop_height_ratio: float,
        lost_timeout_s: float,
        forward_speed: float,
        turn_speed: float,
        target_distance_mm: int = 1000,
        distance_deadband_mm: int = 100,
        slow_range_mm: int = 700,
        tof_filter_alpha: float = 0.30,
    ) -> None:
        self.turn_deadband = max(0.02, min(0.4, turn_deadband))
        self.stop_height_ratio = max(0.2, min(0.9, stop_height_ratio))
        self.lost_timeout_s = max(0.2, lost_timeout_s)
        self.forward_speed = max(0.0, min(1.0, forward_speed))
        self.turn_speed = max(0.0, min(1.0, turn_speed))
        self.target_distance_mm = max(250, target_distance_mm)
        self.distance_deadband_mm = max(30, distance_deadband_mm)
        self.slow_range_mm = max(100, slow_range_mm)
        self.tof_filter_alpha = max(0.05, min(1.0, tof_filter_alpha))
        self.enabled = False
        self.target_id: int | None = None
        self._last_seen_s = None
        self._filtered_distance_mm = None
        self._last_distance_sample_id = None

    def enable(self, target_id: int) -> None:
        self.enabled = True
        self.target_id = target_id
        self._last_seen_s = None
        self._filtered_distance_mm = None
        self._last_distance_sample_id = None

    def disable(self) -> None:
        self.enabled = False
        self.target_id = None
        self._last_seen_s = None
        self._filtered_distance_mm = None
        self._last_distance_sample_id = None

    def command(
        self,
        frame: PersonFrame,
        distance_mm: Optional[int] = None,
        distance_sample_id: Optional[int] = None,
        now_s: Optional[float] = None,
    ) -> tuple[float, float, str]:
        if not self.enabled:
            return 0.0, 0.0, "OFF"
        now = time.monotonic() if now_s is None else now_s
        person = frame.person_by_id(self.target_id)
        if person is not None and now - frame.captured_at <= self.lost_timeout_s:
            self._last_seen_s = now
        elif self._last_seen_s is None or now - self._last_seen_s > self.lost_timeout_s:
            return 0.0, 0.0, "TARGET LOST"
        else:
            return 0.0, 0.0, f"SEARCHING TARGET #{self.target_id}"

        horizontal_error = 0.5 - person.center_x_ratio
        turn = 0.0
        if abs(horizontal_error) > self.turn_deadband:
            scale = min(
                1.0,
                (abs(horizontal_error) - self.turn_deadband) / max(0.01, 0.5 - self.turn_deadband),
            )
            turn = (self.turn_speed * (0.25 + 0.75 * scale)) * (
                1.0 if horizontal_error > 0.0 else -1.0
            )

        if distance_mm is None:
            self._filtered_distance_mm = None
            self._last_distance_sample_id = None
        elif distance_sample_id is None or distance_sample_id != self._last_distance_sample_id:
            sample = float(distance_mm)
            self._filtered_distance_mm = (
                sample
                if self._filtered_distance_mm is None
                else min(
                    sample,
                    self._filtered_distance_mm
                    + self.tof_filter_alpha * (sample - self._filtered_distance_mm),
                )
            )
            self._last_distance_sample_id = distance_sample_id

        if turn:
            return 0.0, turn, f"TARGET #{self.target_id} ALIGNING"
        if person.height_ratio >= self.stop_height_ratio:
            return 0.0, 0.0, f"TARGET #{self.target_id} CAMERA CLOSE"
        if self._filtered_distance_mm is None:
            return 0.0, 0.0, f"TARGET #{self.target_id} TOF WAIT"

        distance = round(self._filtered_distance_mm)
        excess = distance - self.target_distance_mm
        if excess <= 0:
            return 0.0, 0.0, f"TARGET #{self.target_id} HOLD {distance} MM"

        scale = min(1.0, max(0.0, excess - self.distance_deadband_mm) / self.slow_range_mm)
        scale = scale * scale * (3.0 - 2.0 * scale)
        forward = self.forward_speed * (0.45 + 0.55 * scale)
        status = f"TARGET #{self.target_id} FOLLOW {distance} MM"
        return forward, turn, status


class PersonObstaclePlanner:
    """Reactive local planner over the current VL53L5CX 8x8 depth frame."""

    def __init__(
        self,
        stop_distance_mm: int,
        clear_margin_mm: int,
        stable_frames: int,
        turn_speed: float,
    ) -> None:
        self.stop_distance_mm = max(80, stop_distance_mm)
        self.side_clear_mm = self.stop_distance_mm + max(100, clear_margin_mm)
        self.plan_distance_mm = (
            self.stop_distance_mm + max(200, clear_margin_mm * 2) + 100
        )
        self.stable_frames = max(1, stable_frames)
        self.turn_speed = max(0.05, min(1.0, turn_speed))
        self.reset()

    def reset(self) -> None:
        self.direction = 0
        self._near_frames = 0
        self._clear_frames = 0
        self._last_sample_id = None

    @staticmethod
    def _corridors(depth) -> tuple[int | None, int | None, int | None]:
        distances = []
        for first, last in ((0, 3), (2, 6), (5, 8)):
            blocks = []
            for row in (1, 3, 5):
                values = [depth.distances_mm[r * 8 + c]
                          for r in (row, row + 1) for c in range(first, last)]
                valid = [v for v in values if 20 <= v <= 4000]
                if len(valid) < len(values) * 0.75:
                    break
                blocks.append(min(valid))
            distances.append(min(blocks) if len(blocks) == 3 else None)
        return tuple(distances)

    def update(
        self,
        depth,
        forward: float,
        target_turn: float,
    ) -> tuple[float, float, str | None]:
        if depth is None:
            return 0.0, 0.0, "AVOID TOF WAIT"

        obstacle_mm = depth.obstacle_distance_mm
        left_mm, center_mm, right_mm = self._corridors(depth)
        if center_mm is None:
            return 0.0, 0.0, "AVOID TOF WAIT"
        obstacle_mm = min(center_mm, obstacle_mm) if obstacle_mm is not None else center_mm
        left_open = left_mm is not None and left_mm >= self.side_clear_mm
        right_open = right_mm is not None and right_mm >= self.side_clear_mm
        new_sample = depth.sensor_time_ms != self._last_sample_id
        if new_sample:
            self._last_sample_id = depth.sensor_time_ms
            near = obstacle_mm is not None and obstacle_mm <= self.stop_distance_mm
            if self.direction == 0:
                self._near_frames = self._near_frames + 1 if near and forward > 0.0 else 0
                if forward > 0.0 and obstacle_mm is not None and obstacle_mm <= self.stop_distance_mm:
                    self._near_frames = self.stable_frames
                if self._near_frames >= self.stable_frames:
                    if not left_open and not right_open:
                        return 0.0, 0.0, "AVOID BLOCKED"
                    left_score = left_mm or 0
                    right_score = right_mm or 0
                    if not right_open:
                        self.direction = 1
                    elif not left_open:
                        self.direction = -1
                    elif abs(left_score - right_score) < 80 and target_turn:
                        self.direction = 1 if target_turn > 0.0 else -1
                    else:
                        self.direction = 1 if left_score >= right_score else -1
                    self._near_frames = 0
            else:
                center_clear = obstacle_mm >= self.plan_distance_mm
                self._clear_frames = self._clear_frames + 1 if center_clear else 0
                if self._clear_frames >= self.stable_frames:
                    self.reset()
                    return forward, target_turn, None

        if self.direction == 0:
            if self._near_frames >= self.stable_frames and not left_open and not right_open:
                return 0.0, 0.0, "AVOID BLOCKED"
            if obstacle_mm <= self.stop_distance_mm and (forward > 0 or target_turn):
                return 0.0, 0.0, "AVOID BLOCKED"
            return forward, target_turn, None

        if not (left_open if self.direction > 0 else right_open):
            if not new_sample or not (right_open if self.direction > 0 else left_open):
                return 0.0, 0.0, "AVOID BLOCKED"
            self.direction = -self.direction
        front_mm = obstacle_mm
        if front_mm <= self.stop_distance_mm:
            safe_forward = 0.0
        else:
            span = max(1, self.plan_distance_mm - self.stop_distance_mm)
            scale = (front_mm - self.stop_distance_mm) / span
            safe_forward = forward * max(0.30, min(1.0, scale))
        side = "LEFT" if self.direction > 0 else "RIGHT"
        return (
            safe_forward,
            self.direction * self.turn_speed,
            f"AVOID {side} {obstacle_mm if obstacle_mm is not None else front_mm} MM",
        )
