from __future__ import annotations

import json
import math
import statistics
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

from .imu_bno055 import IMUReading


FEATURE_NAMES = (
    "roll_std_deg",
    "pitch_std_deg",
    "roll_range_deg",
    "pitch_range_deg",
    "roll_rate_rms_dps",
    "pitch_rate_rms_dps",
    "roll_rate_peak_dps",
    "pitch_rate_peak_dps",
    "roll_jerk_rms_dps2",
    "pitch_jerk_rms_dps2",
    "gravity_norm_std",
    "swing_roll_asym_deg",
    "swing_pitch_asym_deg",
    "step_rate_hz",
)


@dataclass(frozen=True)
class _Sample:
    time_s: float
    roll_deg: float
    pitch_deg: float
    gravity_norm: Optional[float]
    swing_leg: str
    step_count: int


@dataclass(frozen=True)
class _Profile:
    center: tuple[float, ...]
    scale: tuple[float, ...]
    threshold: float
    samples: int


class GaitAnomalyModel:
    def __init__(self, model_id: str, profiles: dict[str, _Profile]) -> None:
        if "global" not in profiles:
            raise ValueError("Gait anomaly model requires a global profile")
        self.model_id = model_id
        self.profiles = profiles

    @classmethod
    def load(cls, path: Path) -> "GaitAnomalyModel":
        payload = json.loads(path.read_text(encoding="utf-8"))
        if payload.get("version") != 1 or tuple(payload.get("feature_names", ())) != FEATURE_NAMES:
            raise ValueError("Unsupported gait anomaly model format")
        profiles = {}
        for name, raw in payload.get("profiles", {}).items():
            center = tuple(float(value) for value in raw["center"])
            scale = tuple(float(value) for value in raw["scale"])
            threshold = float(raw["threshold"])
            if len(center) != len(FEATURE_NAMES) or len(scale) != len(FEATURE_NAMES):
                raise ValueError(f"Invalid feature count in profile {name}")
            if (
                not all(math.isfinite(value) for value in (*center, *scale, threshold))
                or any(value <= 0.0 for value in scale)
                or threshold <= 0.0
            ):
                raise ValueError(f"Invalid values in profile {name}")
            profiles[str(name)] = _Profile(
                center=center,
                scale=scale,
                threshold=threshold,
                samples=max(0, int(raw.get("samples", 0))),
            )
        return cls(str(payload.get("model_id", "unknown")), profiles)

    def score(self, profile_name: str, features: dict[str, float]) -> tuple[float, float, str]:
        selected_name = profile_name if profile_name in self.profiles else "global"
        profile = self.profiles[selected_name]
        values = [float(features[name]) for name in FEATURE_NAMES]
        distance_sq = sum(
            ((value - center) / scale) ** 2
            for value, center, scale in zip(values, profile.center, profile.scale)
        )
        return math.sqrt(distance_sq / len(FEATURE_NAMES)), profile.threshold, selected_name


class GaitHealthMonitor:
    """Windowed IMU anomaly monitor. It never writes servo commands."""

    ACTIVE_MODES = {"manual", "follow"}
    ACTIVE_PHASES = {"swing", "land"}

    def __init__(
        self,
        *,
        enabled: bool,
        model_path: Path,
        history_path: Path,
        window_s: float = 2.5,
        min_samples: int = 40,
        warning_windows: int = 2,
    ) -> None:
        self.enabled = bool(enabled)
        self.model_path = model_path
        self.history_path = history_path
        self.window_s = max(1.0, float(window_s))
        self.min_samples = max(12, int(min_samples))
        self.warning_windows = max(1, int(warning_windows))
        self.model: GaitAnomalyModel | None = None
        self.model_error: str | None = None
        if self.enabled and self.model_path.is_file():
            try:
                self.model = GaitAnomalyModel.load(self.model_path)
            except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
                self.model_error = str(exc)
        if self.enabled:
            if self.model is not None:
                print(f"[gait-health] Loaded TinyML baseline {self.model.model_id}.")
            elif self.model_error is not None:
                print(f"[gait-health] Model error: {self.model_error}")
            else:
                print("[gait-health] Baseline required; monitoring features without servo control.")

        self._samples: list[_Sample] = []
        self._profile: str | None = None
        self._last_sensor_time_ms: int | None = None
        self._window_id = 0
        self._last_profile: str | None = None
        self._last_features: dict[str, float] | None = None
        self._last_score: float | None = None
        self._last_threshold: float | None = None
        self._last_model_profile: str | None = None
        self._last_status = "BASELINE REQUIRED" if self.model is None else "SAMPLING"
        self._consecutive_anomalies = 0
        self._ratios: deque[float] = deque(maxlen=30)
        self._recent_anomalies: deque[bool] = deque(maxlen=30)
        self._assessed_windows = 0
        self._anomaly_windows = 0
        self._ratio_sum = 0.0
        self._max_ratio = 0.0
        self._profiles_seen: set[str] = set()
        self._maintenance_state = "BASELINE REQUIRED" if self.model is None else "INSUFFICIENT DATA"
        self._history_baseline = self._load_history_baseline()
        self._closed = False

    def update(
        self,
        reading: IMUReading | None,
        gait: dict[str, object],
        runtime_mode: str,
    ) -> dict[str, object]:
        if not self.enabled:
            return self._snapshot("DISABLED")
        if self.model_error is not None:
            self._reset_window()
            return self._snapshot("MODEL ERROR")
        if reading is None:
            self._reset_window()
            return self._snapshot("IMU WAIT")

        phase = str(gait.get("phase", "idle"))
        if runtime_mode not in self.ACTIVE_MODES or phase not in self.ACTIVE_PHASES:
            self._reset_window()
            return self._snapshot("IDLE" if self.model is not None else "BASELINE REQUIRED")

        profile = gait_profile(gait)
        if self._profile is not None and profile != self._profile:
            self._reset_window()
        self._profile = profile

        if reading.sensor_time_ms == self._last_sensor_time_ms:
            return self._snapshot(self._active_status())
        self._last_sensor_time_ms = reading.sensor_time_ms

        sensor_time_s = reading.sensor_time_ms / 1000.0
        if self._samples:
            gap_s = sensor_time_s - self._samples[-1].time_s
            if gap_s <= 0.0 or gap_s > 0.25:
                self._reset_window()
                self._profile = profile

        gravity_norm = None
        if None not in (reading.gravity_x, reading.gravity_y, reading.gravity_z):
            gravity_norm = math.sqrt(
                reading.gravity_x ** 2 + reading.gravity_y ** 2 + reading.gravity_z ** 2
            )
        step_count = _as_int(gait.get("step_count"), 0)
        self._samples.append(
            _Sample(
                time_s=sensor_time_s,
                roll_deg=reading.roll_deg,
                pitch_deg=reading.pitch_deg,
                gravity_norm=gravity_norm,
                swing_leg=str(gait.get("swing_leg") or "none"),
                step_count=step_count,
            )
        )

        duration_s = self._samples[-1].time_s - self._samples[0].time_s
        if duration_s < self.window_s:
            return self._snapshot(self._active_status())
        if len(self._samples) < self.min_samples:
            self._reset_window()
            return self._snapshot("IMU RATE LOW")

        features = extract_features(self._samples)
        self._window_id += 1
        self._last_profile = profile
        self._last_features = features
        self._profiles_seen.add(profile)
        status = "BASELINE REQUIRED"
        if self.model is not None:
            score, threshold, model_profile = self.model.score(profile, features)
            self._last_score = score
            self._last_threshold = threshold
            self._last_model_profile = model_profile
            anomalous = score > threshold
            self._consecutive_anomalies = self._consecutive_anomalies + 1 if anomalous else 0
            ratio = score / threshold
            self._ratios.append(ratio)
            self._recent_anomalies.append(anomalous)
            self._assessed_windows += 1
            self._anomaly_windows += int(anomalous)
            self._ratio_sum += ratio
            self._max_ratio = max(self._max_ratio, ratio)
            self._maintenance_state = self._maintenance_status()
            if self._consecutive_anomalies >= self.warning_windows:
                status = "WARNING"
            elif anomalous:
                status = "SUSPECT"
            else:
                status = "NORMAL"

        self._last_status = status
        self._reset_window()
        return self._snapshot(status)

    def close(self) -> None:
        if self._closed or self.model is None or self._assessed_windows == 0:
            self._closed = True
            return
        self._closed = True
        record = {
            "recorded_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "model_id": self.model.model_id,
            "windows": self._assessed_windows,
            "anomaly_windows": self._anomaly_windows,
            "mean_ratio": self._ratio_sum / self._assessed_windows,
            "max_ratio": self._max_ratio,
            "maintenance_state": self._maintenance_state,
            "profiles": sorted(self._profiles_seen),
        }
        try:
            self.history_path.parent.mkdir(parents=True, exist_ok=True)
            with self.history_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, separators=(",", ":")) + "\n")
        except OSError as exc:
            print(f"[gait-health] Cannot write maintenance history: {exc}")

    def _maintenance_status(self) -> str:
        if not self._ratios:
            return "INSUFFICIENT DATA"
        anomaly_rate = sum(self._recent_anomalies) / len(self._recent_anomalies)
        mean_ratio = statistics.fmean(self._ratios)
        trend = self._trend_percent()
        enough = len(self._ratios) >= 8
        if (
            self._consecutive_anomalies >= self.warning_windows + 1
            or enough and anomaly_rate >= 0.25
        ):
            return "CHECK ROBOT"
        if (
            self._consecutive_anomalies >= self.warning_windows
            or enough and (anomaly_rate >= 0.10 or mean_ratio >= 0.85)
            or enough and trend is not None and trend >= 25.0
        ):
            return "WATCH"
        return "GOOD" if enough else "INSUFFICIENT DATA"

    def _trend_percent(self) -> float | None:
        if self._history_baseline is None or not self._ratios:
            return None
        current = statistics.fmean(self._ratios)
        return (current / self._history_baseline - 1.0) * 100.0

    def _load_history_baseline(self) -> float | None:
        if self.model is None or not self.history_path.is_file():
            return None
        values = []
        try:
            for line in self.history_path.read_text(encoding="utf-8").splitlines()[-30:]:
                record = json.loads(line)
                value = float(record.get("mean_ratio", 0.0))
                if record.get("model_id") == self.model.model_id and value > 0.0 and math.isfinite(value):
                    values.append(value)
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return None
        return statistics.median(values) if values else None

    def _reset_window(self) -> None:
        self._samples.clear()
        self._profile = None

    def _active_status(self) -> str:
        if self.model is None:
            return "BASELINE REQUIRED"
        return self._last_status if self._window_id else "SAMPLING"

    def _snapshot(self, status: str) -> dict[str, object]:
        anomaly_rate = (
            sum(self._recent_anomalies) / len(self._recent_anomalies)
            if self._recent_anomalies
            else None
        )
        return {
            "status": status,
            "model_ready": self.model is not None,
            "model_error": self.model_error,
            "window_id": self._window_id,
            "profile": self._profile or self._last_profile,
            "model_profile": self._last_model_profile,
            "score": self._last_score,
            "threshold": self._last_threshold,
            "features": self._last_features,
            "maintenance": {
                "state": self._maintenance_state,
                "windows": self._assessed_windows,
                "anomaly_rate": anomaly_rate,
                "trend_percent": self._trend_percent(),
            },
        }


def gait_profile(gait: dict[str, object]) -> str:
    commands = gait.get("commands", {})
    if not isinstance(commands, dict):
        return "mixed"
    forward = _as_float(commands.get("forward_mm"))
    turn = _as_float(commands.get("turn_mm"))
    side = _as_float(commands.get("side_mm"))
    magnitudes = {"forward": abs(forward), "turn": abs(turn), "side": abs(side)}
    dominant = max(magnitudes, key=magnitudes.get)
    if magnitudes[dominant] < 0.1:
        return "mixed"
    if dominant == "forward":
        return "forward" if forward >= 0.0 else "backward"
    return dominant


def extract_features(samples: list[_Sample]) -> dict[str, float]:
    if len(samples) < 3:
        raise ValueError("At least three IMU samples are required")
    rolls = [sample.roll_deg for sample in samples]
    pitches = [sample.pitch_deg for sample in samples]
    roll_rates = _rates(samples, "roll_deg")
    pitch_rates = _rates(samples, "pitch_deg")
    roll_jerks = _derivative(roll_rates)
    pitch_jerks = _derivative(pitch_rates)
    gravity = [sample.gravity_norm for sample in samples if sample.gravity_norm is not None]
    duration_s = max(1e-3, samples[-1].time_s - samples[0].time_s)

    return {
        "roll_std_deg": _std(rolls),
        "pitch_std_deg": _std(pitches),
        "roll_range_deg": max(rolls) - min(rolls),
        "pitch_range_deg": max(pitches) - min(pitches),
        "roll_rate_rms_dps": _rms(value for _, value in roll_rates),
        "pitch_rate_rms_dps": _rms(value for _, value in pitch_rates),
        "roll_rate_peak_dps": max(abs(value) for _, value in roll_rates),
        "pitch_rate_peak_dps": max(abs(value) for _, value in pitch_rates),
        "roll_jerk_rms_dps2": _rms(value for _, value in roll_jerks),
        "pitch_jerk_rms_dps2": _rms(value for _, value in pitch_jerks),
        "gravity_norm_std": _std(gravity) if len(gravity) >= 3 else 0.0,
        "swing_roll_asym_deg": _swing_asymmetry(samples, "roll_deg"),
        "swing_pitch_asym_deg": _swing_asymmetry(samples, "pitch_deg"),
        "step_rate_hz": max(0, max(sample.step_count for sample in samples) - min(sample.step_count for sample in samples)) / duration_s,
    }


def _rates(samples: list[_Sample], field: str) -> list[tuple[float, float]]:
    output = []
    for previous, current in zip(samples, samples[1:]):
        dt = current.time_s - previous.time_s
        if dt <= 0.0:
            continue
        delta = _angle_delta(getattr(current, field), getattr(previous, field))
        output.append((current.time_s, delta / dt))
    return output or [(samples[-1].time_s, 0.0)]


def _derivative(values: list[tuple[float, float]]) -> list[tuple[float, float]]:
    output = []
    for previous, current in zip(values, values[1:]):
        dt = current[0] - previous[0]
        if dt > 0.0:
            output.append((current[0], (current[1] - previous[1]) / dt))
    return output or [(values[-1][0], 0.0)]


def _swing_asymmetry(samples: list[_Sample], field: str) -> float:
    center = statistics.fmean(getattr(sample, field) for sample in samples)
    sides = {}
    for side in ("left", "right"):
        values = [getattr(sample, field) - center for sample in samples if sample.swing_leg == side]
        if len(values) < 3:
            return 0.0
        sides[side] = _rms(values)
    return abs(sides["left"] - sides["right"])


def _angle_delta(current: float, previous: float) -> float:
    return (current - previous + 180.0) % 360.0 - 180.0


def _rms(values) -> float:
    sequence = list(values)
    return math.sqrt(statistics.fmean(value * value for value in sequence)) if sequence else 0.0


def _std(values: list[float]) -> float:
    return statistics.pstdev(values) if len(values) >= 2 else 0.0


def _as_float(value: Any, default: float = 0.0) -> float:
    try:
        number = float(value)
        return number if math.isfinite(number) else default
    except (TypeError, ValueError):
        return default


def _as_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default
