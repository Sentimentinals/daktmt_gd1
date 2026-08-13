from __future__ import annotations

import threading
import time
from typing import Optional

from .backends import make_backend
from .balance import BalanceConfig, IMUBalanceController
from .config import Config, STANDING
from .sensors import DepthObstacleGuard, RobotSensorHub, SensorSnapshot
from .terrain_control import TerrainModeController
from .terrain_vision import TerrainKind, TerrainObservation, TerrainPerception
from .walking_engine import DynamicWalkingEngine


class _TerrainCameraWorker:
    def __init__(self, camera, perception: TerrainPerception) -> None:
        self.camera = camera
        self.perception = perception
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._frame = None
        self._observation: Optional[TerrainObservation] = None
        self._updated_at = 0.0
        self._calibrating = True
        self._error: Optional[Exception] = None

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="terrain-camera", daemon=True)
        self._thread.start()

    def set_calibrating(self, enabled: bool) -> None:
        with self._lock:
            self._calibrating = enabled

    def latest(self):
        with self._lock:
            return self._frame, self._observation, self._updated_at, self._error

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                frame = self.camera.capture_array("main")
                with self._lock:
                    calibrating = self._calibrating
                observation = self.perception.update(frame, calibrating=calibrating)
                with self._lock:
                    self._frame = frame
                    self._observation = observation
                    self._updated_at = time.monotonic()
        except Exception as exc:
            with self._lock:
                self._error = exc


def _angle_error(value: float, reference: float) -> float:
    return (value - reference + 180.0) % 360.0 - 180.0


def _sensor_ready(snapshot: SensorSnapshot, args: Config) -> bool:
    imu = snapshot.imu
    return bool(
        imu is not None
        and imu.balance_ready(args.imu_min_gyro_cal, args.imu_min_accel_cal)
        and snapshot.depth is not None
    )


def run_terrain(args: Config) -> None:
    """Continuously classify terrain and run bounded autonomous forward gait."""
    try:
        import cv2
        import pygame
        from libcamera import Transform
        from picamera2 import Picamera2
    except ImportError as exc:
        raise RuntimeError(
            f"Terrain Auto dependency missing: {exc.name}. Install Picamera2/OpenCV from apt."
        ) from exc

    from .keyboard_input import KeyboardReader

    cv2.setNumThreads(1)
    control_hz = max(10, round(1000.0 / args.update_ms))
    reader = KeyboardReader(poll_rate_hz=control_hz)
    reader.init()
    screen = pygame.display.set_mode((args.terrain_camera_width, args.terrain_camera_height))
    pygame.display.set_caption("Terrain Auto")
    font = pygame.font.Font(None, 27)
    small_font = pygame.font.Font(None, 21)

    perception = TerrainPerception(
        stable_frames=args.terrain_stable_frames,
        unknown_frames=args.terrain_unknown_frames,
        calibration_frames=args.terrain_calibration_frames,
        roi_top_ratio=args.terrain_roi_top_ratio,
        horizon_delta_ratio=args.terrain_horizon_delta_ratio,
        horizon_up_sign=args.terrain_horizon_up_sign,
        min_confidence=args.terrain_min_confidence,
    )
    controller = TerrainModeController(
        flat_step_len_mm=args.max_step_len,
        flat_step_height_mm=args.flat_walk_step_height_mm,
        flat_landing_gap_mm=args.landing_gap_mm,
        ramp_step_elevation_mm=args.terrain_ramp_step_elevation_mm,
        stair_rise_mm=args.terrain_stair_rise_mm,
        stair_tread_mm=args.terrain_stair_tread_mm,
        min_confidence=args.terrain_min_confidence,
        allow_stairs_down=args.terrain_allow_stairs_down,
        stair_depth_relief_mm=args.terrain_stair_depth_relief_mm,
    )
    obstacle_guard = DepthObstacleGuard(
        stop_distance_mm=args.tof_obstacle_stop_mm,
        clear_margin_mm=args.tof_obstacle_clear_margin_mm,
        stable_frames=args.tof_obstacle_stable_frames,
    )
    engine = DynamicWalkingEngine(
        dt=args.update_ms / 1000.0,
        t_step=args.terrain_t_step,
        t_dbl=args.terrain_t_dbl,
        max_step_len=args.max_step_len,
        max_turn_step_len=0.0,
        max_side_step_len=0.0,
        step_height=args.step_height,
        zmp_support_ratio=args.zmp_support_ratio,
        ankle_roll_gain=args.ankle_roll_gain,
        step_x_ratio=args.step_x_ratio,
        landing_gap_mm=args.landing_gap_mm,
        lift_start_phase=args.lift_start_phase,
        swing_advance_end_phase=args.swing_advance_end_phase,
        lift_end_phase=args.lift_end_phase,
        landing_roll_release_start=args.landing_roll_release_start,
        command_rate_limit=min(args.command_rate_limit, 12.0),
        arm_swing_pwm=args.arm_swing_pwm,
        arm_right_dir=args.arm_right_dir,
        arm_left_dir=args.arm_left_dir,
        arm_smooth_tau=args.arm_smooth_tau,
        arm_min_pwm=args.arm_min_pwm,
        arm_quantum_pwm=args.arm_quantum_pwm,
        max_step_elevation=args.terrain_max_step_elevation_mm,
    )
    sensor_hub = RobotSensorHub(
        port=args.sensor_port,
        baudrate=args.sensor_baudrate,
        timeout_s=args.sensor_timeout_s,
        depth_timeout_s=args.sensor_depth_timeout_s,
        use_imu=True,
        use_foot_fsr=args.sensor_use_foot_fsr,
        use_depth=True,
        imu_roll_sign=args.imu_roll_sign,
        imu_pitch_sign=args.imu_pitch_sign,
        imu_yaw_sign=args.imu_yaw_sign,
        imu_vertical_mount=args.imu_vertical_mount,
        imu_board_face_sign=args.imu_board_face_sign,
        foot_fsr_invert=args.foot_fsr_invert,
        foot_fsr_filter_alpha=args.foot_fsr_filter_alpha,
        foot_fsr_zero_raw=args.foot_fsr_zero_raw,
        foot_fsr_full_raw=args.foot_fsr_full_raw,
    )
    sensor_hub.open()

    camera = Picamera2()
    camera.configure(
        camera.create_preview_configuration(
            main={
                "format": "RGB888",
                "size": (args.terrain_camera_width, args.terrain_camera_height),
            },
            controls={"FrameRate": args.terrain_camera_fps},
            transform=Transform(hflip=True, vflip=True),
        )
    )
    camera_worker = _TerrainCameraWorker(camera, perception)
    backend = make_backend(mode=args.backend, port=args.port, baudrate=args.baudrate, csv_path=args.csv)

    armed = False
    fault_latched = False
    fault_reason = ""
    previous_toggle = False
    previous_stop = False
    reference: Optional[tuple[float, float]] = None
    balance: Optional[IMUBalanceController] = None
    last_balance_t = time.monotonic()

    try:
        camera.start()
        camera_worker.start()
        with backend:
            backend.send(STANDING, duration_ms=900, force=True)
            time.sleep(1.0)
            print("[terrain] Keep robot upright on flat ground while IMU and camera calibrate.")
            reference = sensor_hub.capture_imu_reference(
                sample_seconds=args.imu_reference_seconds,
                timeout_s=args.imu_reference_timeout_s,
                min_gyro_cal=args.imu_min_gyro_cal,
                min_accel_cal=args.imu_min_accel_cal,
                max_rms_deg=args.imu_reference_max_rms_deg,
            )
            if reference is not None:
                balance = IMUBalanceController(
                    BalanceConfig(
                        target_roll_deg=reference[0],
                        target_pitch_deg=reference[1],
                        max_correction_deg=args.terrain_balance_limit_deg,
                        roll_deadband_deg=args.terrain_balance_deadband_deg,
                        pitch_deadband_deg=args.terrain_balance_deadband_deg,
                    )
                )
            else:
                print("[terrain] IMU reference failed. Preview works, but autonomous gait cannot arm.")

            print("[terrain] V toggles AUTO, C clears to standing, O/Escape returns to menu.")
            for state in reader.poll():
                if state.quit or state.menu:
                    print("[terrain] Returning to function menu.")
                    break

                frame, observation, camera_at, camera_error = camera_worker.latest()
                camera_worker.set_calibrating(not armed and not fault_latched)
                snapshot = sensor_hub.read()
                sensors_ok = _sensor_ready(snapshot, args)
                camera_ok = camera_error is None and frame is not None and time.monotonic() - camera_at <= 0.65
                profile = None
                status = "WAITING FOR CAMERA"
                if observation is not None:
                    profile, status = controller.select(observation, snapshot.depth)
                terrain_close_allowed = bool(
                    observation is not None
                    and observation.kind
                    in (TerrainKind.RAMP_UP, TerrainKind.RAMP_DOWN, TerrainKind.STAIRS_UP, TerrainKind.STAIRS_DOWN)
                )
                stop_distance = (
                    args.tof_terrain_emergency_stop_mm
                    if terrain_close_allowed
                    else args.tof_obstacle_stop_mm
                )
                obstacle_blocked, obstacle_mm = obstacle_guard.update(snapshot.depth, stop_distance)

                toggle = state.auto_toggle
                if toggle and not previous_toggle:
                    if armed:
                        armed = False
                        print("[terrain] AUTO OFF.")
                    elif fault_latched:
                        print(f"[terrain] Fault is latched: {fault_reason}. Press C after supporting robot.")
                    elif reference is None:
                        print("[terrain] Cannot arm: IMU reference is unavailable.")
                    elif not sensors_ok:
                        print("[terrain] Cannot arm: IMU feedback is not ready.")
                    elif not camera_ok or profile is None:
                        print("[terrain] Cannot arm: terrain is not stable or camera calibration is incomplete.")
                    else:
                        armed = True
                        engine.reset()
                        print(f"[terrain] AUTO ON: {profile.label}.")
                previous_toggle = toggle

                stop_pressed = state.stop
                if stop_pressed and not previous_stop:
                    armed = False
                    fault_latched = False
                    fault_reason = ""
                    engine.reset()
                    if balance is not None:
                        balance.reset()
                    backend.send(STANDING, duration_ms=800, force=True)
                    print("[terrain] Reset to STANDING. AUTO is OFF.")
                previous_stop = stop_pressed

                if armed and (not camera_ok or not sensors_ok):
                    fault_reason = "CAMERA LOST" if not camera_ok else "SENSOR STREAM LOST"
                    fault_latched = True
                    armed = False

                imu = snapshot.imu
                if armed and reference is not None and imu is not None:
                    roll_error = abs(_angle_error(imu.roll_deg, reference[0]))
                    pitch_error = abs(_angle_error(imu.pitch_deg, reference[1]))
                    if max(roll_error, pitch_error) > args.terrain_emergency_tilt_deg:
                        fault_reason = f"TILT {max(roll_error, pitch_error):.1f} DEG"
                        fault_latched = True
                        armed = False

                if fault_latched:
                    pose = engine.update(0.0)
                    if engine.is_idle_ready():
                        pose = dict(STANDING)
                    status = f"FAULT: {fault_reason}"
                else:
                    command = 0.0
                    step_elevation = 0.0
                    if armed and obstacle_blocked:
                        status = f"OBJECT {obstacle_mm} MM - HOLD"
                    elif armed and profile is not None:
                        controller.apply(engine, profile)
                        command = profile.command
                        step_elevation = profile.step_elevation_mm
                        status = profile.label
                    pose = engine.update(command, step_elevation_mm=step_elevation)

                    if balance is not None and imu is not None:
                        now = time.monotonic()
                        dt = now - last_balance_t
                        last_balance_t = now
                        support_leg = engine.support_leg
                        pose = balance.apply(
                            pose,
                            roll_deg=imu.roll_deg,
                            pitch_deg=imu.pitch_deg,
                            dt=dt,
                            support_leg=support_leg,
                        )

                backend.send(pose, duration_ms=args.update_ms)
                if frame is not None and observation is not None:
                    preview = perception.draw(frame, observation)
                    preview = cv2.cvtColor(cv2.flip(preview, 1), cv2.COLOR_BGR2RGB)
                    surface = pygame.surfarray.make_surface(preview.swapaxes(0, 1))
                    screen.blit(surface, (0, 0))
                    state_label = "FAULT" if fault_latched else ("AUTO" if armed else "AUTO OFF")
                    confidence = round(observation.confidence * 100)
                    label = font.render(f"{state_label}  {status}", True, (245, 90, 90) if fault_latched else (70, 235, 165))
                    detail = small_font.render(
                        f"vision {confidence}%  sensors {'OK' if sensors_ok else 'WAIT'}  "
                        f"ToF {obstacle_mm if obstacle_mm is not None else '--'} mm",
                        True,
                        (235, 238, 242),
                    )
                    panel = pygame.Surface((max(label.get_width(), detail.get_width()) + 24, 58), pygame.SRCALPHA)
                    panel.fill((10, 14, 18, 210))
                    screen.blit(panel, (12, 12))
                    screen.blit(label, (24, 19))
                    screen.blit(detail, (24, 45))
                    pygame.display.flip()

            if not fault_latched:
                backend.send(STANDING, duration_ms=args.stop_ms, force=True)
                time.sleep(args.stop_ms / 1000.0)
            else:
                print(f"[terrain] Fault remains latched: {fault_reason}. Support robot and press C before power-off.")
    except KeyboardInterrupt:
        print("\n[terrain] Interrupted. AUTO stopped.")
    finally:
        camera_worker.stop()
        try:
            camera.stop()
        except Exception:
            pass
        try:
            camera.close()
        except Exception:
            pass
        sensor_hub.close()
        reader.quit()
