from __future__ import annotations

from dataclasses import dataclass

from .terrain_vision import TerrainKind, TerrainObservation


@dataclass(frozen=True)
class TerrainProfile:
    label: str
    command: float
    max_step_len_mm: float
    step_height_mm: float
    step_elevation_mm: float
    landing_gap_mm: float


class TerrainModeController:
    """Map stable visual terrain classes to bounded gait parameters."""

    def __init__(
        self,
        flat_step_len_mm: float,
        flat_step_height_mm: float,
        flat_landing_gap_mm: float,
        ramp_step_elevation_mm: float,
        stair_rise_mm: float,
        stair_tread_mm: float,
        min_confidence: float,
        allow_stairs_down: bool,
    ) -> None:
        stair_rise_mm = max(1.0, abs(stair_rise_mm))
        ramp_step_elevation_mm = max(0.5, abs(ramp_step_elevation_mm))
        stair_tread_mm = max(40.0, abs(stair_tread_mm))
        self.min_confidence = min_confidence
        self.allow_stairs_down = allow_stairs_down
        self.profiles = {
            TerrainKind.FLAT: TerrainProfile(
                "FLAT",
                0.44,
                min(flat_step_len_mm, 28.0),
                flat_step_height_mm,
                0.0,
                flat_landing_gap_mm,
            ),
            TerrainKind.RAMP_UP: TerrainProfile(
                "RAMP UP",
                0.34,
                min(flat_step_len_mm, 22.0),
                max(flat_step_height_mm, 34.0),
                ramp_step_elevation_mm,
                min(flat_landing_gap_mm, 58.0),
            ),
            TerrainKind.RAMP_DOWN: TerrainProfile(
                "RAMP DOWN",
                0.26,
                min(flat_step_len_mm, 18.0),
                max(24.0, flat_step_height_mm * 0.90),
                -ramp_step_elevation_mm,
                min(flat_landing_gap_mm, 52.0),
            ),
            TerrainKind.STAIRS_UP: TerrainProfile(
                "STAIRS UP",
                0.50,
                min(24.0, stair_tread_mm / 1.72),
                max(38.0, stair_rise_mm + 22.0),
                stair_rise_mm,
                stair_tread_mm,
            ),
            TerrainKind.STAIRS_DOWN: TerrainProfile(
                "STAIRS DOWN",
                0.34,
                min(20.0, stair_tread_mm / 1.72),
                max(30.0, stair_rise_mm + 14.0),
                -stair_rise_mm,
                stair_tread_mm,
            ),
        }

    def select(self, observation: TerrainObservation) -> tuple[TerrainProfile | None, str]:
        if not observation.calibrated:
            return None, "CALIBRATING"
        if observation.kind == TerrainKind.UNKNOWN or observation.confidence < self.min_confidence:
            return None, "TERRAIN UNKNOWN"
        if observation.kind == TerrainKind.STAIRS_DOWN and not self.allow_stairs_down:
            return None, "STAIRS DOWN LOCKED"
        profile = self.profiles.get(observation.kind)
        return (profile, profile.label) if profile is not None else (None, "TERRAIN UNKNOWN")

    @staticmethod
    def apply(engine, profile: TerrainProfile) -> None:
        engine.max_step_len = profile.max_step_len_mm
        engine.step_height = profile.step_height_mm
        engine.landing_gap_mm = profile.landing_gap_mm
