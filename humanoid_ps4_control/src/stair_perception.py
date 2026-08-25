from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from statistics import median
from typing import Optional

from .sensors import DepthReading


@dataclass(frozen=True)
class StairDetection:
    box: tuple[int, int, int, int]
    confidence: float
    direction: str
    center_error: float
    line_count: int
    source: str


@dataclass(frozen=True)
class StairFrame:
    stairs: tuple[StairDetection, ...] = ()
    captured_at: float = 0.0

    @property
    def primary_stair(self) -> Optional[StairDetection]:
        return self.stairs[0] if self.stairs else None


@dataclass(frozen=True)
class StairGeometry:
    direction: str
    confidence: float
    edge_distance_mm: Optional[int]
    riser_height_mm: float
    center_error: float
    source: str


class StairDetector:
    def __init__(
        self,
        model_path: str,
        confidence: float = 0.55,
        iou_threshold: float = 0.45,
        input_size: int = 416,
        detect_every_frames: int = 3,
    ) -> None:
        import cv2

        self._cv2 = cv2
        self.confidence = max(0.05, min(0.95, confidence))
        self.iou_threshold = max(0.1, min(0.9, iou_threshold))
        self.input_size = max(160, int(input_size) // 32 * 32)
        self.detect_every_frames = max(1, detect_every_frames)
        self._frame_count = 0
        self._last = StairFrame()
        model = Path(model_path)
        self._net = cv2.dnn.readNetFromONNX(str(model)) if model.is_file() else None

    @property
    def model_ready(self) -> bool:
        return self._net is not None

    def detect(self, frame) -> StairFrame:
        self._frame_count += 1
        if self._frame_count % self.detect_every_frames:
            return self._last
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("Stair detector expects a three-channel camera frame")

        model_detection = self._detect_model(frame) if self._net is not None else None
        line_detection = self._detect_lines(frame)
        if model_detection is not None and line_detection is not None:
            overlap = self._box_iou(model_detection.box, line_detection.box)
            confidence = min(0.99, model_detection.confidence + 0.12 * overlap)
            detection = StairDetection(
                box=model_detection.box,
                confidence=confidence,
                direction=model_detection.direction,
                center_error=model_detection.center_error,
                line_count=line_detection.line_count,
                source="model+lines",
            )
        else:
            detection = model_detection or line_detection

        stairs = (detection,) if detection is not None else ()
        self._last = StairFrame(stairs, time.monotonic())
        return self._last

    def _detect_model(self, frame) -> Optional[StairDetection]:
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
        if rows.ndim != 2:
            return None
        if rows.shape[0] in (5, 6, 7):
            rows = rows.T
        if rows.shape[1] not in (5, 6, 7):
            return None

        boxes: list[list[int]] = []
        scores: list[float] = []
        directions: list[str] = []
        for row in rows:
            class_scores = row[4:]
            class_id = int(class_scores.argmax())
            score = float(class_scores[class_id])
            if score < self.confidence:
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
            scores.append(score)
            directions.append("up" if class_id == 0 and len(class_scores) > 1 else "down" if class_id == 1 else "unknown")

        if not boxes:
            return None
        indices = cv2.dnn.NMSBoxes(boxes, scores, self.confidence, self.iou_threshold)
        kept = [int(item[0]) if hasattr(item, "__len__") else int(item) for item in indices]
        if not kept:
            return None
        index = max(kept, key=lambda item: scores[item] * boxes[item][2] * boxes[item][3])
        x, y, box_width, box_height = boxes[index]
        box = (x, y, x + box_width, y + box_height)
        center_error = ((x + box_width * 0.5) / max(1, width) - 0.5) * 2.0
        return StairDetection(box, scores[index], directions[index], center_error, 0, "model")

    def _detect_lines(self, frame) -> Optional[StairDetection]:
        cv2 = self._cv2
        height, width = frame.shape[:2]
        roi_top = round(height * 0.16)
        gray = cv2.cvtColor(frame[roi_top:], cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gray, 55, 145)
        raw_lines = cv2.HoughLinesP(
            edges,
            1,
            math.pi / 180.0,
            threshold=max(24, width // 14),
            minLineLength=max(40, round(width * 0.22)),
            maxLineGap=max(10, width // 28),
        )
        if raw_lines is None:
            return None

        lines = []
        for raw in raw_lines.reshape(-1, 4):
            x1, y1, x2, y2 = (int(value) for value in raw)
            angle = abs(math.degrees(math.atan2(y2 - y1, x2 - x1)))
            angle = min(angle, abs(180.0 - angle))
            length = math.hypot(x2 - x1, y2 - y1)
            if angle <= 11.0 and length >= width * 0.22:
                lines.append((x1, y1 + roi_top, x2, y2 + roi_top, length))
        if len(lines) < 4:
            return None

        lines.sort(key=lambda item: (item[1] + item[3]) * 0.5)
        separated = []
        for line in lines:
            center_y = (line[1] + line[3]) * 0.5
            if separated and abs(center_y - (separated[-1][1] + separated[-1][3]) * 0.5) < 7:
                if line[4] > separated[-1][4]:
                    separated[-1] = line
            else:
                separated.append(line)
        if len(separated) < 4:
            return None

        centers = [(line[1] + line[3]) * 0.5 for line in separated]
        gaps = [b - a for a, b in zip(centers, centers[1:]) if b - a >= 5]
        if len(gaps) < 3:
            return None
        gap_mean = sum(gaps) / len(gaps)
        gap_error = sum(abs(gap - gap_mean) for gap in gaps) / max(1.0, len(gaps) * gap_mean)
        consistency = max(0.0, min(1.0, 1.0 - gap_error))
        x1 = max(0, min(min(line[0], line[2]) for line in separated))
        x2 = min(width, max(max(line[0], line[2]) for line in separated))
        y1 = max(0, round(min(centers) - gap_mean))
        y2 = min(height, round(max(centers) + gap_mean))
        if x2 - x1 < width * 0.28 or y2 - y1 < height * 0.10:
            return None
        line_score = min(1.0, (len(separated) - 3) / 6.0)
        confidence = 0.38 + 0.30 * line_score + 0.22 * consistency
        center_error = ((x1 + x2) * 0.5 / max(1, width) - 0.5) * 2.0
        return StairDetection(
            (x1, y1, x2, y2),
            min(0.88, confidence),
            "unknown",
            center_error,
            len(separated),
            "lines",
        )

    @staticmethod
    def _box_iou(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
        x1 = max(a[0], b[0])
        y1 = max(a[1], b[1])
        x2 = min(a[2], b[2])
        y2 = min(a[3], b[3])
        intersection = max(0, x2 - x1) * max(0, y2 - y1)
        area_a = max(0, a[2] - a[0]) * max(0, a[3] - a[1])
        area_b = max(0, b[2] - b[0]) * max(0, b[3] - b[1])
        return intersection / max(1, area_a + area_b - intersection)


def estimate_stair_geometry(
    detection: StairDetection,
    depth: DepthReading | None,
    *,
    default_riser_mm: float,
    min_riser_mm: float,
    max_riser_mm: float,
    mount_height_mm: float,
    pitch_down_deg: float,
    vertical_fov_deg: float,
    flip_vertical: bool,
    direction_delta_mm: float,
) -> StairGeometry:
    direction = "unknown"
    edge_distance = depth.tracking_distance_mm if depth is not None else None
    riser_height = default_riser_mm
    source = detection.source

    if depth is not None:
        rows = [depth.region_median_mm(row, row + 1, 1, 7) for row in range(8)]
        if flip_vertical:
            rows.reverse()
        top = [value for value in rows[:3] if value is not None]
        bottom = [value for value in rows[5:] if value is not None]
        if top and bottom:
            delta = median(top) - median(bottom)
            if abs(delta) >= direction_delta_mm:
                direction = "up" if delta > 0 else "down"

        surface_heights = []
        for row, distance in enumerate(rows):
            if distance is None:
                continue
            ray_offset = ((row + 0.5) / 8.0 - 0.5) * vertical_fov_deg
            ray_down = math.radians(pitch_down_deg + ray_offset)
            surface_heights.append(mount_height_mm - distance * math.sin(ray_down))
        if len(surface_heights) >= 4:
            low = median(sorted(surface_heights)[:2])
            high = median(sorted(surface_heights)[-2:])
            measured = abs(high - low)
            if min_riser_mm <= measured <= max_riser_mm:
                riser_height = measured
        source += "+tof"

    confidence = detection.confidence
    if depth is not None:
        confidence = min(0.99, confidence + 0.08)
    if direction == "unknown":
        confidence *= 0.72
    return StairGeometry(
        direction=direction,
        confidence=confidence,
        edge_distance_mm=edge_distance,
        riser_height_mm=max(min_riser_mm, min(max_riser_mm, riser_height)),
        center_error=detection.center_error,
        source=source,
    )
