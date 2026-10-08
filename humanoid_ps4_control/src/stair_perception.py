from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from statistics import median
from typing import Optional

from .sensors import DepthReading
from .vision_yolo import detect_yolo


@dataclass(frozen=True)
class StairDetection:
    box: tuple[int, int, int, int]
    confidence: float
    center_error: float
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
    edge_uncertainty_mm: float = 0.0
    riser_measured: bool = False


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
        self._net = None
        if model.is_file():
            try:
                self._net = cv2.dnn.readNetFromONNX(str(model))
            except cv2.error as exc:
                print(f"[stair] ONNX unavailable; using line detection: {exc}")

    @property
    def model_ready(self) -> bool:
        return self._net is not None

    def detect(self, frame, *, captured_at: float | None = None) -> StairFrame:
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3 or frame.size == 0:
            raise ValueError("Stair detector expects a non-empty three-channel camera frame")
        captured_at = time.monotonic() if captured_at is None else captured_at
        self._frame_count += 1
        if self._frame_count % self.detect_every_frames:
            return self._last

        model_detection = self._detect_model(frame) if self._net is not None else None
        line_detection = self._detect_lines(frame)
        # Model confidence alone is not evidence of visible tread edges.
        detection = line_detection
        if model_detection is not None and line_detection is not None:
            overlap = self._box_iou(model_detection.box, line_detection.box)
            if overlap > 0.0:
                confidence = min(0.99, model_detection.confidence + 0.12 * overlap)
                detection = StairDetection(
                    box=model_detection.box,
                    confidence=confidence,
                    center_error=model_detection.center_error,
                    source="model+lines",
                )

        stairs = (detection,) if detection is not None else ()
        self._last = StairFrame(stairs, captured_at)
        return self._last

    def _detect_model(self, frame) -> Optional[StairDetection]:
        width = frame.shape[1]
        try:
            detections = detect_yolo(
                self._cv2,
                self._net,
                frame,
                class_count=1,
                input_size=self.input_size,
                confidence=self.confidence,
                iou_threshold=self.iou_threshold,
            )
        except (self._cv2.error, RuntimeError) as exc:
            self._net = None
            print(f"[stair] ONNX inference unavailable; using line detection: {exc}")
            return None
        if not detections:
            return None
        detected = max(detections, key=lambda item: item.confidence * item.area_ratio)
        x1, _, x2, _ = detected.box
        box_width = x2 - x1
        center_error = ((x1 + box_width * 0.5) / max(1, width) - 0.5) * 2.0
        return StairDetection(detected.box, detected.confidence, center_error, "model")

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
        if not lines:
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
        if not separated or len(separated) > 12:
            return None
        # Tread edges share a horizontal span; unrelated floor/noise lines do not.
        common_left = max(min(line[0], line[2]) for line in separated)
        common_right = min(max(line[0], line[2]) for line in separated)
        if common_right - common_left < width * 0.28:
            return None

        centers = [(line[1] + line[3]) * 0.5 for line in separated]
        if len(separated) == 1:
            if separated[0][4] < width * 0.38 or centers[0] < height * 0.62:
                return None
            gap_mean = height * 0.06
            consistency = 0.55
        else:
            gaps = [b - a for a, b in zip(centers, centers[1:]) if b - a >= 5]
            if not gaps:
                return None
            gap_mean = sum(gaps) / len(gaps)
            gap_error = sum(abs(gap - gap_mean) for gap in gaps) / max(1.0, len(gaps) * gap_mean)
            consistency = max(0.0, min(1.0, 1.0 - gap_error))
        x1 = max(0, min(min(line[0], line[2]) for line in separated))
        x2 = min(width, max(max(line[0], line[2]) for line in separated))
        y1 = max(0, round(min(centers) - gap_mean))
        y2 = min(height, round(max(centers) + gap_mean))
        if (
            x2 - x1 < width * 0.28
            or y2 - y1 < height * 0.035
            or max(centers) < height * 0.52
        ):
            return None
        line_score = min(1.0, len(separated) / 6.0)
        coverage = min(1.0, (x2 - x1) / max(1.0, width * 0.60))
        confidence = 0.36 + 0.18 * line_score + 0.18 * consistency + 0.10 * coverage
        center_error = ((x1 + x2) * 0.5 / max(1, width) - 0.5) * 2.0
        return StairDetection(
            (x1, y1, x2, y2),
            min(0.88, confidence),
            center_error,
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
    detection: StairDetection | None,
    depth: DepthReading | None,
    *,
    default_riser_mm: float,
    min_riser_mm: float,
    max_riser_mm: float,
    mount_height_mm: float,
    pitch_down_deg: float,
    vertical_fov_deg: float,
    flip_vertical: bool,
    forward_offset_mm: float = 0.0,
    roll_deg: float = 0.0,
) -> StairGeometry:
    direction = "unknown"
    edge_distance = None
    edge_uncertainty = 0.0
    riser_height = default_riser_mm
    source = detection.source if detection is not None else "no-camera"

    if depth is not None and len(depth.distances_mm) == 64 and all(math.isfinite(v) for v in (
        mount_height_mm, pitch_down_deg, vertical_fov_deg, roll_deg, forward_offset_mm,
    )) and mount_height_mm > 0 and 0 < vertical_fov_deg < 90:
        pitch = math.radians(pitch_down_deg)
        roll = math.radians(roll_deg)
        half_fov = math.tan(math.radians(vertical_fov_deg * 0.5))
        points = []
        tolerance = min(5.0, default_riser_mm * 0.25)
        for row in range(8):
            sensor_row = 7 - row if flip_vertical else row
            samples = []
            for col in (3, 4):
                distance = depth.distances_mm[sensor_row * 8 + col]
                if not 20 <= distance <= 4000:
                    continue
                # ULD distance_mm is axial depth, already radial-to-perpendicular
                # compensated by ST. Reconstruct a zone, then rotate to the floor.
                down = distance * ((row + 0.5) / 4.0 - 1.0) * half_fov
                right = distance * ((col + 0.5) / 4.0 - 1.0) * half_fov
                down = down * math.cos(roll) - right * math.sin(roll)
                forward = distance * math.cos(pitch) - down * math.sin(pitch)
                height = mount_height_mm - distance * math.sin(pitch) - down * math.cos(pitch)
                if forward > 0 and height < mount_height_mm:
                    samples.append((forward + forward_offset_mm, height))
            if len(samples) == 2 and abs(samples[0][1] - samples[1][1]) <= 2 * tolerance:
                points.append((median(p[0] for p in samples), median(p[1] for p in samples)))
        points.sort()
        # Raw range gradients also occur on flat ground. Require two horizontal
        # levels, each supported by multiple zones, with the near level at floor.
        for split in range(2, len(points) - 1):
            near = points[:split]
            far = points[split:split + 2]
            far_level = median(p[1] for p in far)
            for point in points[split + 2:]:
                if abs(point[1] - far_level) > tolerance:
                    break
                far.append(point)
            near_height = median(p[1] for p in near)
            far_height = median(p[1] for p in far)
            delta = far_height - near_height
            if abs(near_height) > tolerance:
                continue
            if not min_riser_mm <= abs(delta) <= max_riser_mm:
                continue
            if any(abs(p[1] - near_height) > tolerance for p in near):
                continue
            if any(abs(p[1] - far_height) > tolerance for p in far):
                continue
            if any(
                group[-1][0] - group[0][0] < 20.0
                or abs(group[-1][1] - group[0][1]) / (group[-1][0] - group[0][0]) > 0.06
                for group in (near, far)
            ):
                continue
            near_x, far_x = near[-1][0], far[0][0]
            if far_x <= near_x or near[0][0] <= 0:
                continue
            direction = "up" if delta > 0 else "down"
            riser_height = abs(delta)
            edge_distance = round((near_x + far_x) * 0.5)
            edge_uncertainty = (far_x - near_x) * 0.5 + tolerance
            break
        if direction != "unknown":
            source += "+tof-levels"
        else:
            source += "+tof-unresolved"

    confidence = detection.confidence if detection is not None else 0.0
    if direction == "unknown":
        confidence *= 0.72
    return StairGeometry(
        direction=direction,
        confidence=confidence,
        edge_distance_mm=edge_distance,
        riser_height_mm=max(min_riser_mm, min(max_riser_mm, riser_height)),
        center_error=detection.center_error if detection is not None else 0.0,
        source=source,
        edge_uncertainty_mm=edge_uncertainty,
        riser_measured=direction != "unknown",
    )
