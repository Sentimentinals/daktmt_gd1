from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from .vision_yolo import detect_yolo


@dataclass(frozen=True)
class ObjectDetection:
    label: str
    color: str
    box: tuple[int, int, int, int]
    confidence: float
    area_ratio: float


@dataclass(frozen=True)
class ObjectFrame:
    objects: tuple[ObjectDetection, ...] = ()
    captured_at: float = 0.0

    @property
    def primary_object(self) -> Optional[ObjectDetection]:
        return self.objects[0] if self.objects else None


class PickupObjectDetector:
    CLASS_NAMES = ("CAN", "BALL", "RUBIK CUBE")
    COLOR_RANGES = {
        "RED": (((0, 90, 55), (8, 255, 255)), ((170, 90, 55), (179, 255, 255))),
        "ORANGE": (((9, 90, 55), (21, 255, 255)),),
        "YELLOW": (((22, 80, 65), (35, 255, 255)),),
        "GREEN": (((36, 65, 45), (85, 255, 255)),),
        "BLUE": (((86, 70, 45), (130, 255, 255)),),
        "PURPLE": (((131, 65, 45), (169, 255, 255)),),
    }

    def __init__(
        self,
        model_path: str,
        confidence: float = 0.55,
        iou_threshold: float = 0.45,
        input_size: int = 416,
        detect_every_frames: int = 3,
    ) -> None:
        import cv2

        model = Path(model_path)
        if not model.is_file():
            raise FileNotFoundError(f"Pickup detector model missing: {model}")
        self._cv2 = cv2
        self._net = cv2.dnn.readNetFromONNX(str(model))
        self.confidence = max(0.05, min(0.95, confidence))
        self.iou_threshold = max(0.1, min(0.9, iou_threshold))
        self.input_size = max(160, int(input_size) // 32 * 32)
        self.detect_every_frames = max(1, detect_every_frames)
        self._frame_count = 0
        self._last = ObjectFrame()

    def detect(self, frame) -> ObjectFrame:
        self._frame_count += 1
        if self._frame_count % self.detect_every_frames:
            return self._last
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("Pickup detector expects a three-channel BGR frame")

        detected_boxes = detect_yolo(
            self._cv2,
            self._net,
            frame,
            class_count=len(self.CLASS_NAMES),
            input_size=self.input_size,
            confidence=self.confidence,
            iou_threshold=self.iou_threshold,
        )
        objects = [
            ObjectDetection(
                label=self.CLASS_NAMES[detected.class_id],
                color=self._dominant_color(frame, detected.box),
                box=detected.box,
                confidence=detected.confidence,
                area_ratio=detected.area_ratio,
            )
            for detected in detected_boxes[:4]
        ]
        self._last = ObjectFrame(tuple(objects), time.monotonic())
        return self._last

    def _dominant_color(self, frame, box: tuple[int, int, int, int]) -> str:
        x1, y1, x2, y2 = box
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            return ""
        hsv = self._cv2.cvtColor(roi, self._cv2.COLOR_BGR2HSV)
        counts = {}
        for color, ranges in self.COLOR_RANGES.items():
            count = 0
            for lower, upper in ranges:
                count += self._cv2.countNonZero(self._cv2.inRange(hsv, lower, upper))
            counts[color] = count
        min_pixels = roi.shape[0] * roi.shape[1] * 0.08
        present = [color for color, count in counts.items() if count >= min_pixels]
        if len(present) >= 3:
            return "MULTI"
        color, count = max(counts.items(), key=lambda item: item[1])
        return color if count >= min_pixels else ""
