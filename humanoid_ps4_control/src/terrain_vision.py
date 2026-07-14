from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from enum import Enum
from statistics import median
from typing import Deque, Iterable

import cv2
import numpy as np


class TerrainKind(str, Enum):
    UNKNOWN = "unknown"
    FLAT = "flat"
    RAMP_UP = "ramp_up"
    RAMP_DOWN = "ramp_down"
    STAIRS_UP = "stairs_up"
    STAIRS_DOWN = "stairs_down"


@dataclass(frozen=True)
class TerrainObservation:
    kind: TerrainKind
    candidate: TerrainKind
    confidence: float
    horizon_ratio: float | None
    horizontal_levels: tuple[int, ...]
    edge_density: float
    calibrated: bool


class TerrainPerception:
    """Lightweight monocular terrain classifier for a fixed forward camera."""

    def __init__(
        self,
        stable_frames: int = 6,
        unknown_frames: int = 3,
        calibration_frames: int = 24,
        roi_top_ratio: float = 0.28,
        horizon_delta_ratio: float = 0.055,
        horizon_up_sign: float = 1.0,
        min_confidence: float = 0.55,
    ) -> None:
        self.stable_frames = max(2, stable_frames)
        self.unknown_frames = max(1, unknown_frames)
        self.calibration_frames = max(8, calibration_frames)
        self.roi_top_ratio = max(0.10, min(0.65, roi_top_ratio))
        self.horizon_delta_ratio = max(0.015, min(0.20, horizon_delta_ratio))
        self.horizon_up_sign = 1.0 if horizon_up_sign >= 0.0 else -1.0
        self.min_confidence = max(0.30, min(0.90, min_confidence))

        self._flat_horizons: Deque[float] = deque(maxlen=self.calibration_frames)
        self._flat_horizon_ratio: float | None = None
        self._calibration_seen = 0
        self._pending = TerrainKind.UNKNOWN
        self._pending_count = 0
        self._unknown_count = 0
        self._stable = TerrainKind.UNKNOWN
        self._stable_confidence = 0.0

    @property
    def calibrated(self) -> bool:
        return self._flat_horizon_ratio is not None

    def reset_state(self) -> None:
        self._pending = TerrainKind.UNKNOWN
        self._pending_count = 0
        self._unknown_count = 0
        self._stable = TerrainKind.UNKNOWN
        self._stable_confidence = 0.0

    def update(self, frame: np.ndarray, calibrating: bool = False) -> TerrainObservation:
        height, width = frame.shape[:2]
        y0 = round(height * self.roi_top_ratio)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(6, 6)).apply(gray)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gray, 55, 145)
        edges[:y0, :] = 0

        lines_raw = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180.0,
            threshold=max(24, width // 15),
            minLineLength=max(24, width // 10),
            maxLineGap=max(8, width // 30),
        )
        lines = (
            []
            if lines_raw is None
            else [tuple(map(int, item)) for item in np.asarray(lines_raw).reshape(-1, 4)]
        )
        horizontal = self._horizontal_lines(lines, width)
        levels = self._cluster_levels(horizontal, max(6, height // 45))
        horizon_ratio, horizon_support = self._vanishing_horizon(lines, width, height)
        roi_edges = edges[y0:, :]
        edge_density = float(np.count_nonzero(roi_edges)) / max(1, roi_edges.size)

        if calibrating and self._flat_horizon_ratio is None and len(levels) < 3:
            self._calibration_seen += 1
            if horizon_ratio is not None and horizon_support >= 2:
                self._flat_horizons.append(horizon_ratio)
            enough_samples = len(self._flat_horizons) >= max(6, self.calibration_frames // 2)
            if self._calibration_seen >= self.calibration_frames and enough_samples:
                self._flat_horizon_ratio = float(median(self._flat_horizons))
            elif self._calibration_seen >= self.calibration_frames * 2 and self._flat_horizon_ratio is None:
                self._flat_horizon_ratio = 0.45

        candidate, confidence = self._classify(
            levels=levels,
            horizontal=horizontal,
            horizon_ratio=horizon_ratio,
            horizon_support=horizon_support,
            edge_density=edge_density,
            frame_height=height,
        )
        stable, stable_confidence = self._stabilize(candidate, confidence)
        return TerrainObservation(
            kind=stable,
            candidate=candidate,
            confidence=stable_confidence,
            horizon_ratio=horizon_ratio,
            horizontal_levels=tuple(levels),
            edge_density=edge_density,
            calibrated=self.calibrated,
        )

    def draw(self, frame: np.ndarray, observation: TerrainObservation) -> np.ndarray:
        out = frame.copy()
        height, width = out.shape[:2]
        y0 = round(height * self.roi_top_ratio)
        cv2.line(out, (0, y0), (width - 1, y0), (80, 190, 255), 1)
        for y in observation.horizontal_levels:
            cv2.line(out, (round(width * 0.12), y), (round(width * 0.88), y), (60, 220, 150), 2)
        if observation.horizon_ratio is not None:
            y = round(observation.horizon_ratio * height)
            cv2.line(out, (0, y), (width - 1, y), (255, 180, 60), 1)
        return out

    def _classify(
        self,
        levels: list[int],
        horizontal: list[tuple[int, int, int, int]],
        horizon_ratio: float | None,
        horizon_support: int,
        edge_density: float,
        frame_height: int,
    ) -> tuple[TerrainKind, float]:
        delta = None
        if horizon_ratio is not None and self._flat_horizon_ratio is not None:
            delta = (horizon_ratio - self._flat_horizon_ratio) * self.horizon_up_sign

        if edge_density > 0.16:
            return TerrainKind.UNKNOWN, 0.0

        if len(levels) >= 3:
            spacings = np.diff(np.asarray(levels, dtype=float))
            spacing_mean = float(np.mean(spacings)) if len(spacings) else 0.0
            spacing_cv = float(np.std(spacings) / max(1.0, spacing_mean))
            regularity = max(0.0, 1.0 - spacing_cv / 0.85)
            coverage = min(1.0, len(horizontal) / max(4.0, len(levels) * 1.5))
            confidence = min(0.98, 0.48 + 0.10 * min(4, len(levels) - 2) + 0.22 * regularity * coverage)
            if delta is not None and abs(delta) >= self.horizon_delta_ratio * 0.55:
                return (TerrainKind.STAIRS_UP if delta > 0.0 else TerrainKind.STAIRS_DOWN), confidence

            level_center = float(np.mean(levels)) / max(1, frame_height)
            fallback = TerrainKind.STAIRS_UP if level_center >= 0.58 else TerrainKind.STAIRS_DOWN
            return fallback, confidence * 0.82

        if delta is not None and horizon_support >= 2 and abs(delta) >= self.horizon_delta_ratio:
            excess = (abs(delta) - self.horizon_delta_ratio) / self.horizon_delta_ratio
            confidence = min(0.94, 0.58 + 0.16 * excess + 0.04 * min(4, horizon_support))
            return (TerrainKind.RAMP_UP if delta > 0.0 else TerrainKind.RAMP_DOWN), confidence

        if not self.calibrated:
            return TerrainKind.UNKNOWN, 0.0

        if delta is not None and abs(delta) <= self.horizon_delta_ratio * 0.55:
            return TerrainKind.FLAT, min(0.92, 0.68 + 0.04 * min(4, horizon_support))

        if horizon_ratio is None and len(levels) <= 1 and edge_density < 0.045:
            return TerrainKind.FLAT, 0.62

        return TerrainKind.UNKNOWN, max(0.0, 0.45 - edge_density * 3.0)

    def _stabilize(self, candidate: TerrainKind, confidence: float) -> tuple[TerrainKind, float]:
        if confidence < self.min_confidence:
            candidate = TerrainKind.UNKNOWN

        if candidate == TerrainKind.UNKNOWN:
            self._unknown_count += 1
            self._pending = TerrainKind.UNKNOWN
            self._pending_count = 0
            if self._unknown_count >= self.unknown_frames:
                self._stable = TerrainKind.UNKNOWN
                self._stable_confidence = 0.0
            return self._stable, self._stable_confidence

        self._unknown_count = 0
        if candidate != self._pending:
            self._pending = candidate
            self._pending_count = 1
        else:
            self._pending_count += 1

        if candidate == self._stable:
            self._stable_confidence += 0.25 * (confidence - self._stable_confidence)
        elif self._pending_count >= self.stable_frames:
            self._stable = candidate
            self._stable_confidence = confidence
        return self._stable, self._stable_confidence

    @staticmethod
    def _horizontal_lines(
        lines: Iterable[tuple[int, int, int, int]], width: int
    ) -> list[tuple[int, int, int, int]]:
        result = []
        for x1, y1, x2, y2 in lines:
            dx = x2 - x1
            dy = y2 - y1
            length = math.hypot(dx, dy)
            angle = abs(math.degrees(math.atan2(dy, dx)))
            angle = min(angle, 180.0 - angle)
            if length >= width * 0.22 and angle <= 12.0:
                result.append((x1, y1, x2, y2))
        return result

    @staticmethod
    def _cluster_levels(lines: Iterable[tuple[int, int, int, int]], tolerance: int) -> list[int]:
        values = sorted(round((y1 + y2) * 0.5) for _, y1, _, y2 in lines)
        clusters: list[list[int]] = []
        for value in values:
            if not clusters or value - round(sum(clusters[-1]) / len(clusters[-1])) > tolerance:
                clusters.append([value])
            else:
                clusters[-1].append(value)
        return [round(sum(cluster) / len(cluster)) for cluster in clusters]

    @staticmethod
    def _vanishing_horizon(
        lines: Iterable[tuple[int, int, int, int]], width: int, height: int
    ) -> tuple[float | None, int]:
        negative = []
        positive = []
        for line in lines:
            x1, y1, x2, y2 = line
            dx = x2 - x1
            dy = y2 - y1
            if abs(dx) < 2:
                continue
            angle = math.degrees(math.atan2(dy, dx))
            folded = angle if -90.0 <= angle <= 90.0 else angle - math.copysign(180.0, angle)
            if 16.0 <= abs(folded) <= 72.0 and math.hypot(dx, dy) >= width * 0.12:
                (positive if folded > 0.0 else negative).append(line)

        intersections = []
        for first in negative[:8]:
            for second in positive[:8]:
                point = TerrainPerception._intersection(first, second)
                if point is None:
                    continue
                x, y = point
                if -0.25 * width <= x <= 1.25 * width and -0.35 * height <= y <= 0.90 * height:
                    intersections.append(y / height)
        if len(intersections) < 2:
            return None, len(intersections)
        return float(median(intersections)), len(intersections)

    @staticmethod
    def _intersection(
        first: tuple[int, int, int, int], second: tuple[int, int, int, int]
    ) -> tuple[float, float] | None:
        x1, y1, x2, y2 = first
        x3, y3, x4, y4 = second
        denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denominator) < 1e-6:
            return None
        determinant_a = x1 * y2 - y1 * x2
        determinant_b = x3 * y4 - y3 * x4
        x = (determinant_a * (x3 - x4) - (x1 - x2) * determinant_b) / denominator
        y = (determinant_a * (y3 - y4) - (y1 - y2) * determinant_b) / denominator
        return x, y
