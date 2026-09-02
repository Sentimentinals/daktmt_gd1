from __future__ import annotations

import threading
import time
from typing import Optional

from .balance import (
    configured_fall_detector,
    extend_arms_forward,
    update_fall_detector,
)
from .config import Config, STANDING


class PriorityBackend:
    """Serialize servo writes and let a fall pose override every mode."""

    def __init__(self, backend, arm_forward_pwm: int) -> None:
        self._backend = backend
        self._arm_forward_pwm = arm_forward_pwm
        self._lock = threading.RLock()
        self._pose = dict(STANDING)
        self._fall_pose: Optional[dict[int, int]] = None

    def __enter__(self) -> "PriorityBackend":
        return self

    def __exit__(self, *_) -> None:
        return None

    @property
    def current_pose(self) -> dict[int, int]:
        with self._lock:
            return dict(self._fall_pose or self._pose)

    def send(self, pose: dict[int, int], duration_ms: int = 1000, force: bool = False) -> None:
        with self._lock:
            command = self._fall_pose if self._fall_pose is not None else pose
            self._backend.send(command, duration_ms=duration_ms, force=force)
            if self._fall_pose is None:
                self._pose = dict(pose)

    def trigger_fall(self, duration_ms: int) -> None:
        with self._lock:
            if self._fall_pose is None:
                self._fall_pose = extend_arms_forward(self._pose, self._arm_forward_pwm)
                self._backend.send(self._fall_pose, duration_ms=duration_ms, force=True)

    def release_fall(self, duration_ms: int, return_to_standing: bool) -> None:
        with self._lock:
            if self._fall_pose is None:
                return
            self._fall_pose = None
            if return_to_standing:
                self._pose = dict(STANDING)
                self._backend.send(STANDING, duration_ms=duration_ms, force=True)


class FallSafety:
    """Persistent IMU fall detector shared by all dashboard modes."""

    def __init__(self, args: Config, sensor_hub, backend: PriorityBackend) -> None:
        self.args = args
        self.sensor_hub = sensor_hub
        self.backend = backend
        self._detector = configured_fall_detector(args)
        self._reference: Optional[tuple[float, float]] = None
        self._active = False
        self._reason = ""
        self._imu_live = False
        self._suspended = False
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active

    @property
    def reference(self) -> Optional[tuple[float, float]]:
        with self._lock:
            return self._reference

    @property
    def reason(self) -> str:
        with self._lock:
            return self._reason

    @property
    def status(self) -> str:
        if not self.args.fall_detection_enabled:
            return "FALL OFF"
        if self.sensor_hub is None:
            return "FALL IMU UNAVAILABLE"
        with self._lock:
            if self._suspended:
                return "FALL RECOVERY"
            if self._active:
                return f"FALL ACTIVE: {self._reason}"
            if self._reference is None:
                return "FALL IMU WAIT"
            return "FALL READY" if self._imu_live else "FALL IMU STALE"

    def start(self) -> None:
        if (
            self._thread is not None
            or self.sensor_hub is None
            or not (self.args.fall_detection_enabled or self.args.imu_balance)
        ):
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="fall-safety",
            daemon=True,
        )
        self._thread.start()

    def begin_recovery(self) -> None:
        with self._lock:
            self._suspended = True
            self._active = False
            self._reason = ""
            self._detector.reset()
            self.backend.release_fall(self.args.update_ms, return_to_standing=False)

    def end_recovery(self) -> None:
        with self._lock:
            self._detector.reset()
            self._suspended = False

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=self.args.imu_reference_timeout_s + 1.0)
        self._thread = None

    def _run(self) -> None:
        last_update = time.monotonic()
        while not self._stop.is_set():
            with self._lock:
                suspended = self._suspended
                reference = self._reference
            if suspended:
                self._stop.wait(0.03)
                last_update = time.monotonic()
                continue

            if reference is None:
                captured = self.sensor_hub.capture_imu_reference(
                    sample_seconds=self.args.imu_reference_seconds,
                    timeout_s=self.args.imu_reference_timeout_s,
                    min_gyro_cal=self.args.imu_min_gyro_cal,
                    min_accel_cal=self.args.imu_min_accel_cal,
                    max_rms_deg=self.args.imu_reference_max_rms_deg,
                    cancel_event=self._stop,
                )
                if captured is None:
                    self._stop.wait(0.5)
                    continue
                with self._lock:
                    self._reference = captured
                    self._detector.reset()
                reference = captured
                last_update = time.monotonic()
                print(
                    f"[fall] Ready at roll={captured[0]:.2f}, "
                    f"pitch={captured[1]:.2f}; monitoring every dashboard mode."
                )

            snapshot = self.sensor_hub.read()
            reading = snapshot.imu
            now = time.monotonic()
            with self._lock:
                self._imu_live = reading is not None
                suspended = self._suspended
            if suspended or reading is None:
                self._stop.wait(0.03)
                last_update = now
                continue
            if not self.args.fall_detection_enabled:
                self._stop.wait(max(0.01, self.args.update_ms / 1000.0))
                last_update = now
                continue

            with self._lock:
                if self._suspended:
                    last_update = now
                    continue
                was_active = self._detector.triggered
                active = update_fall_detector(
                    self._detector,
                    reading,
                    reference,
                    now - last_update,
                    self.args,
                )
                reason = self._detector.reason
                self._active = active
                self._reason = reason
                if active and not was_active:
                    self.backend.trigger_fall(self.args.update_ms)
                elif was_active and not active:
                    self.backend.release_fall(self.args.stop_ms, return_to_standing=True)
            last_update = now

            if active and not was_active:
                print(f"[fall] FALL detected: {reason}. All mode commands are blocked.")
            elif was_active and not active:
                print("[fall] IMU upright again. Returned to STANDING.")

            self._stop.wait(max(0.01, self.args.update_ms / 1000.0))
