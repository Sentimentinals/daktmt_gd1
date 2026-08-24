from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


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

        cv2 = self._cv2
        height, width = frame.shape[:2]
        scale = min(self.input_size / width, self.input_size / height)
        resized_width = max(1, round(width * scale))
        resized_height = max(1, round(height * scale))
        resized = cv2.resize(frame, (resized_width, resized_height))
        pad_x = (self.input_size - resized_width) // 2
        pad_y = (self.input_size - resized_height) // 2
        padded = cv2.copyMakeBorder(
            resized,
            pad_y,
            self.input_size - resized_height - pad_y,
            pad_x,
            self.input_size - resized_width - pad_x,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )
        blob = cv2.dnn.blobFromImage(
            padded,
            scalefactor=1.0 / 255.0,
            size=(self.input_size, self.input_size),
            swapRB=True,
            crop=False,
        )
        self._net.setInput(blob)
        raw = self._net.forward()
        rows = raw[0]
        expected_columns = 4 + len(self.CLASS_NAMES)
        if rows.ndim != 2:
            raise RuntimeError(f"Unsupported pickup model output shape: {raw.shape}")
        if rows.shape[0] == expected_columns:
            rows = rows.T
        if rows.shape[1] != expected_columns:
            raise RuntimeError(f"Unsupported pickup model output shape: {raw.shape}")

        boxes = []
        scores = []
        class_ids = []
        for row in rows:
            class_scores = row[4:]
            class_id = int(class_scores.argmax())
            confidence = float(class_scores[class_id])
            if confidence < self.confidence:
                continue
            center_x, center_y, box_width, box_height = (float(value) for value in row[:4])
            x = round((center_x - box_width * 0.5 - pad_x) / scale)
            y = round((center_y - box_height * 0.5 - pad_y) / scale)
            box_width = round(box_width / scale)
            box_height = round(box_height / scale)
            x1 = max(0, min(width - 1, x))
            y1 = max(0, min(height - 1, y))
            x2 = max(0, min(width, x + box_width))
            y2 = max(0, min(height, y + box_height))
            if x2 <= x1 or y2 <= y1:
                continue
            boxes.append([x1, y1, x2 - x1, y2 - y1])
            scores.append(confidence)
            class_ids.append(class_id)

        objects = []
        indices = cv2.dnn.NMSBoxes(boxes, scores, self.confidence, self.iou_threshold)
        for raw_index in indices:
            index = int(raw_index[0]) if hasattr(raw_index, "__len__") else int(raw_index)
            x, y, box_width, box_height = boxes[index]
            box = (x, y, x + box_width, y + box_height)
            objects.append(
                ObjectDetection(
                    label=self.CLASS_NAMES[class_ids[index]],
                    color=self._dominant_color(frame, box),
                    box=box,
                    confidence=scores[index],
                    area_ratio=(box_width * box_height) / float(max(1, width * height)),
                )
            )

        objects.sort(key=lambda item: (item.confidence, item.area_ratio), reverse=True)
        self._last = ObjectFrame(tuple(objects[:4]), time.monotonic())
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
