from __future__ import annotations

import math
import time
from dataclasses import dataclass
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


class SimpleObjectDetector:
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
        min_area_ratio: float = 0.01,
        max_area_ratio: float = 0.72,
        detect_every_frames: int = 2,
    ) -> None:
        import cv2

        self._cv2 = cv2
        self.min_area_ratio = max(0.002, min(0.20, min_area_ratio))
        self.max_area_ratio = max(self.min_area_ratio, min(0.95, max_area_ratio))
        self.detect_every_frames = max(1, detect_every_frames)
        self._frame_count = 0
        self._last = ObjectFrame()

    def detect(self, frame) -> ObjectFrame:
        self._frame_count += 1
        if self._frame_count % self.detect_every_frames:
            return self._last

        cv2 = self._cv2
        height, width = frame.shape[:2]
        frame_area = float(max(1, width * height))
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        masks = {}
        combined = None
        for color, ranges in self.COLOR_RANGES.items():
            mask = None
            for lower, upper in ranges:
                part = cv2.inRange(hsv, lower, upper)
                mask = part if mask is None else cv2.bitwise_or(mask, part)
            masks[color] = mask
            combined = mask.copy() if combined is None else cv2.bitwise_or(combined, mask)

        join_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
        clean_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, join_kernel)
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, clean_kernel)
        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        objects = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            area_ratio = area / frame_area
            if not self.min_area_ratio <= area_ratio <= self.max_area_ratio:
                continue

            x, y, box_width, box_height = cv2.boundingRect(contour)
            if box_width < 8 or box_height < 8:
                continue
            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 0.0:
                continue

            aspect = box_width / float(box_height)
            elongation = max(aspect, 1.0 / max(0.01, aspect))
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            box_area = float(box_width * box_height)
            color_pixels = {
                color: cv2.countNonZero(mask[y : y + box_height, x : x + box_width])
                for color, mask in masks.items()
            }
            present_colors = [
                color for color, count in color_pixels.items()
                if count >= max(30, round(box_area * 0.035))
            ]
            dominant_color = max(color_pixels, key=color_pixels.get)

            label = None
            confidence = 0.0
            if len(present_colors) >= 3 and 0.65 <= aspect <= 1.55:
                label = "RUBIK CUBE"
                dominant_color = "MULTI"
                confidence = min(0.98, 0.62 + 0.07 * len(present_colors))
            elif 0.72 <= aspect <= 1.38 and circularity >= 0.68:
                label = "BALL"
                confidence = min(0.96, 0.52 + 0.50 * circularity)
            elif elongation >= 1.25:
                label = "CAN"
                confidence = min(0.92, 0.58 + 0.10 * min(3.0, elongation - 1.0))
            if label is None:
                continue

            objects.append(
                ObjectDetection(
                    label=label,
                    color=dominant_color,
                    box=(x, y, x + box_width, y + box_height),
                    confidence=confidence,
                    area_ratio=area_ratio,
                )
            )

        objects.sort(key=lambda item: (item.area_ratio, item.confidence), reverse=True)
        self._last = ObjectFrame(tuple(objects[:4]), time.monotonic())
        return self._last
