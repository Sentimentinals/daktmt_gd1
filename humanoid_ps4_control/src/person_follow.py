from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from .sensors import DepthReading


@dataclass(frozen=True)
class PersonDetection:
    box: tuple[int, int, int, int]
    confidence: float
    frame_width: int
    frame_height: int

    @property
    def center_x_ratio(self) -> float:
        x1, _, x2, _ = self.box
        return ((x1 + x2) * 0.5) / max(1, self.frame_width)

    @property
    def height_ratio(self) -> float:
        _, y1, _, y2 = self.box
        return max(0.0, y2 - y1) / max(1, self.frame_height)


@dataclass(frozen=True)
class ObjectDetection:
    label: str
    box: tuple[int, int, int, int]
    confidence: float
    frame_width: int
    frame_height: int

    @property
    def center_x_ratio(self) -> float:
        x1, _, x2, _ = self.box
        return ((x1 + x2) * 0.5) / max(1, self.frame_width)

    @property
    def bottom_y_ratio(self) -> float:
        return self.box[3] / max(1, self.frame_height)


@dataclass(frozen=True)
class PersonFrame:
    people: tuple[PersonDetection, ...] = ()
    captured_at: float = 0.0
    objects: tuple[ObjectDetection, ...] = ()

    @property
    def single_person(self) -> Optional[PersonDetection]:
        return self.people[0] if len(self.people) == 1 else None

    @property
    def target_object(self) -> Optional[ObjectDetection]:
        if not self.objects:
            return None
        return max(
            self.objects,
            key=lambda item: item.confidence + 0.20 * item.bottom_y_ratio
            - 0.25 * abs(item.center_x_ratio - 0.5),
        )


class PersonDetector:
    PERSON_CLASS_ID = 15
    CLASS_NAMES = (
        "background",
        "aeroplane",
        "bicycle",
        "bird",
        "boat",
        "bottle",
        "bus",
        "car",
        "cat",
        "chair",
        "cow",
        "diningtable",
        "dog",
        "horse",
        "motorbike",
        "person",
        "pottedplant",
        "sheep",
        "sofa",
        "train",
        "tvmonitor",
    )

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
        objects = []
        for index in range(raw.shape[2]):
            confidence = float(raw[0, 0, index, 2])
            class_id = int(raw[0, 0, index, 1])
            if confidence < self.confidence or not 0 < class_id < len(self.CLASS_NAMES):
                continue
            values = raw[0, 0, index, 3:7]
            x1 = max(0, min(width - 1, int(values[0] * width)))
            y1 = max(0, min(height - 1, int(values[1] * height)))
            x2 = max(0, min(width - 1, int(values[2] * width)))
            y2 = max(0, min(height - 1, int(values[3] * height)))
            if x2 <= x1 or y2 <= y1:
                continue
            if class_id == self.PERSON_CLASS_ID:
                people.append(
                    PersonDetection(
                        box=(x1, y1, x2, y2),
                        confidence=confidence,
                        frame_width=width,
                        frame_height=height,
                    )
                )
            else:
                objects.append(
                    ObjectDetection(
                        label=self.CLASS_NAMES[class_id],
                        box=(x1, y1, x2, y2),
                        confidence=confidence,
                        frame_width=width,
                        frame_height=height,
                    )
                )

        people.sort(key=lambda person: person.confidence, reverse=True)
        objects.sort(key=lambda item: item.confidence, reverse=True)
        self._last = PersonFrame(tuple(people), time.monotonic(), tuple(objects))
        return self._last


class SquatTargetController:
    def __init__(
        self,
        min_distance_mm: int,
        max_distance_mm: int,
        camera_center_tolerance: float,
        min_depth_ratio: float,
        target_timeout_s: float,
    ) -> None:
        self.min_distance_mm = max(80, min_distance_mm)
        self.max_distance_mm = max(self.min_distance_mm + 50, max_distance_mm)
        self.camera_center_tolerance = max(0.05, min(0.45, camera_center_tolerance))
        self.min_depth_ratio = max(0.0, min(0.8, min_depth_ratio))
        self.target_timeout_s = max(0.2, target_timeout_s)

    def command(
        self,
        depth: DepthReading | None,
        frame: PersonFrame,
        now_s: Optional[float] = None,
    ) -> tuple[float, str]:
        now = time.monotonic() if now_s is None else now_s
        if depth is None:
            return 0.0, "TOF WAIT"
        target = frame.target_object
        if target is None or now - frame.captured_at > self.target_timeout_s:
            return 0.0, "OBJECT WAIT"
        if abs(target.center_x_ratio - 0.5) > self.camera_center_tolerance:
            return 0.0, "CENTER OBJECT"

        distance = depth.obstacle_distance_mm
        if distance is None:
            return 0.0, "TOF INVALID"
        if distance < self.min_distance_mm:
            return 0.0, "OBJECT TOO CLOSE"
        if distance > self.max_distance_mm:
            return 0.0, "OBJECT TOO FAR"

        camera_ratio = max(0.0, min(1.0, (target.bottom_y_ratio - 0.40) / 0.55))
        distance_ratio = (self.max_distance_mm - distance) / (
            self.max_distance_mm - self.min_distance_mm
        )
        ratio = 0.70 * camera_ratio + 0.30 * distance_ratio
        ratio = max(self.min_depth_ratio, min(1.0, ratio))
        return ratio, f"{target.label.upper()} {distance} MM"


class PersonFollowController:
    def __init__(
        self,
        turn_deadband: float,
        stop_height_ratio: float,
        lost_timeout_s: float,
        forward_speed: float,
        turn_speed: float,
    ) -> None:
        self.turn_deadband = max(0.02, min(0.4, turn_deadband))
        self.stop_height_ratio = max(0.2, min(0.9, stop_height_ratio))
        self.lost_timeout_s = max(0.2, lost_timeout_s)
        self.forward_speed = max(0.0, min(1.0, forward_speed))
        self.turn_speed = max(0.0, min(1.0, turn_speed))
        self.enabled = False
        self._last_seen_s = None

    def enable(self) -> None:
        self.enabled = True
        self._last_seen_s = None

    def disable(self) -> None:
        self.enabled = False
        self._last_seen_s = None

    def command(
        self,
        frame: PersonFrame,
        now_s: Optional[float] = None,
    ) -> tuple[float, float, str]:
        if not self.enabled:
            return 0.0, 0.0, "OFF"
        now = time.monotonic() if now_s is None else now_s
        if len(frame.people) > 1:
            return 0.0, 0.0, "MULTIPLE PEOPLE"
        person = frame.single_person
        if person is not None and now - frame.captured_at <= self.lost_timeout_s:
            self._last_seen_s = now
        elif self._last_seen_s is None or now - self._last_seen_s > self.lost_timeout_s:
            return 0.0, 0.0, "TARGET LOST"
        else:
            return 0.0, 0.0, "SEARCHING"

        horizontal_error = 0.5 - person.center_x_ratio
        turn = 0.0
        if abs(horizontal_error) > self.turn_deadband:
            turn = self.turn_speed if horizontal_error > 0.0 else -self.turn_speed

        forward = 0.0
        if person.height_ratio < self.stop_height_ratio:
            forward = self.forward_speed
        status = "FOLLOWING" if forward or turn else "FOLLOW DISTANCE"
        return forward, turn, status
