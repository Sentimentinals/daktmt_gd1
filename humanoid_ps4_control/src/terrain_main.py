from __future__ import annotations

import threading
import time
from typing import Optional

from .backends import make_backend
from .balance import BalanceConfig, IMUBalanceController
from .config import Config, STANDING
from .sensors import RobotSensorHub, SensorSnapshot
from .terrain_control import TerrainModeController
from .terrain_vision import TerrainObservation, TerrainPerception
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
    load = snapshot.foot_load
    return bool(
        imu is not None
        and imu.balance_ready(args.imu_min_gyro_cal, args.imu_min_accel_cal)
        and load is not None
        and load.total >= args.fsr_min_total_load
    )


def run_terrain(args: Config) -> None:
    """Continuously classify terrain and run bounded autonomous forward gait."""
    try:
        import cv2
        import pygame
        from picamera2 import Picamera2
    except ImportError as exc:
        raise RuntimeError(
            f"Terrain Auto dependency missing: {exc.name}. Install Picamera2/OpenCV from apt."
        ) from exc

    from .ps4_pygame import PS4Reader

    cv2.setNumThreads(1)
    control_hz = max(10, round(1000.0 / args.update_ms))
    reader = PS4Reader(
        joystick_index=args.joystick_index,
        fallback_keys=True,
        poll_rate_hz=control_hz,
        deadzone=args.input_deadzone,
    )
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
        flat_step_height_mm=args.step_height,
        flat_landing_gap_mm=args.landing_gap_mm,
        ramp_step_elevation_mm=args.terrain_ramp_step_elevation_mm,
        stair_rise_mm=args.terrain_stair_rise_mm,
        stair_tread_mm=args.terrain_stair_tread_mm,
        min_confidence=args.terrain_min_confidence,
        allow_stairs_down=args.terrain_allow_stairs_down,
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
        hip_abduct_gain=args.hip_abduct_gain,
        ankle_roll_gain=args.ankle_roll_gain,
        step_x_ratio=args.step_x_ratio,
        left_swing_x_scale=args.left_swing_x_scale,
        left_step_height_scale=args.left_step_height_scale,
        landing_gap_mm=args.landing_gap_mm,
        right_swing_x_scale=args.right_swing_x_scale,
        right_step_height_scale=args.right_step_height_scale,
        lift_start_phase=args.lift_start_phase,
        swing_advance_end_phase=args.swing_advance_end_phase,
        lift_end_phase=args.lift_end_phase,
        landing_roll_release_start=args.landing_roll_release_start,
        command_rate_limit=min(args.command_rate_limit, 12.0),
        arm_swing_pwm=args.arm_swing_pwm,
        arm_right_dir=args.arm_right_dir,
        arm_left_dir=args.arm_left_dir,
        arm_elbow_ratio=args.arm_elbow_ratio,
        arm_lift_ratio=args.arm_lift_ratio,
        arm_smooth_tau=args.arm_smooth_tau,
        arm_min_pwm=args.arm_min_pwm,
        arm_quantum_pwm=args.arm_quantum_pwm,
        max_step_elevation=args.terrain_max_step_elevation_mm,
    )
    engine.stop_extra_steps = 0

    sensor_hub = RobotSensorHub(
        port=args.sensor_port,
        baudrate=args.sensor_baudrate,
        timeout_s=args.sensor_timeout_s,
        use_imu=True,
        use_fsr=True,
        imu_roll_sign=args.imu_roll_sign,
        imu_pitch_sign=args.imu_pitch_sign,
        imu_yaw_sign=args.imu_yaw_sign,
        imu_vertical_mount=args.imu_vertical_mount,
        imu_board_face_sign=args.imu_board_face_sign,
        fsr_invert=args.fsr_invert,
        fsr_filter_alpha=args.fsr_filter_alpha,
        fsr_left_zero_raw=args.fsr_left_zero_raw,
        fsr_left_full_raw=args.fsr_left_full_raw,
        fsr_right_zero_raw=args.fsr_right_zero_raw,
        fsr_right_full_raw=args.fsr_right_full_raw,
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
        )
    )
    camera_worker = _TerrainCameraWorker(camera, perception)
    backend = make_backend(mode=args.backend, port=args.port, baudrate=args.baudrate, csv_path=args.csv)

    armed = False
    fault_latched = False
    fault_reason = ""
    previous_toggle = False
    previous_stop = False
    last_pose = dict(STANDING)
    reference: Optional[tuple[float, float]] = None
    balance: Optional[IMUBalanceController] = None
    last_balance_t = time.monotonic()
    support_invalid_frames = 0
    touchdown_invalid_frames = 0

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
                        max_correction_deg=args.balance_limit_deg,
                        roll_deadband_deg=args.balance_deadband_deg,
                        pitch_deadband_deg=args.balance_deadband_deg,
                    )
                )
            else:
                print("[terrain] IMU reference failed. Preview works, but autonomous gait cannot arm.")

            print("[terrain] Square toggles AUTO, Circle clears to standing, Options returns to menu.")
            for state in reader.poll():
                if state.quit or state.button(reader.BTN_OPTIONS):
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
                    profile, status = controller.select(observation)

                toggle = state.button(reader.BTN_SQUARE)
                if toggle and not previous_toggle:
                    if armed:
                        armed = False
                        print("[terrain] AUTO OFF.")
                    elif fault_latched:
                        print(f"[terrain] Fault is latched: {fault_reason}. Press Circle after supporting robot.")
                    elif reference is None:
                        print("[terrain] Cannot arm: IMU reference is unavailable.")
                    elif not sensors_ok:
                        print("[terrain] Cannot arm: IMU/FSR feedback is not ready.")
                    elif not camera_ok or profile is None:
                        print("[terrain] Cannot arm: terrain is not stable or camera calibration is incomplete.")
                    else:
                        armed = True
                        engine.reset()
                        print(f"[terrain] AUTO ON: {profile.label}.")
                previous_toggle = toggle

                stop_pressed = state.button(reader.BTN_CIRCLE)
                if stop_pressed and not previous_stop:
                    armed = False
                    fault_latched = False
                    fault_reason = ""
                    support_invalid_frames = 0
                    touchdown_invalid_frames = 0
                    engine.reset()
                    last_pose = dict(STANDING)
                    backend.send(STANDING, duration_ms=800, force=True)
                    print("[terrain] Reset to STANDING. AUTO is OFF.")
                previous_stop = stop_pressed

                if armed and (not camera_ok or not sensors_ok):
                    fault_reason = "CAMERA LOST" if not camera_ok else "IMU/FSR LOST"
                    fault_latched = True
                    armed = False

                imu = snapshot.imu
                load = snapshot.foot_load
                if armed and reference is not None and imu is not None:
                    roll_error = abs(_angle_error(imu.roll_deg, reference[0]))
                    pitch_error = abs(_angle_error(imu.pitch_deg, reference[1]))
                    if max(roll_error, pitch_error) > args.terrain_emergency_tilt_deg:
                        fault_reason = f"TILT {max(roll_error, pitch_error):.1f} DEG"
                        fault_latched = True
                        armed = False

                if armed and load is not None and engine.support_leg in ("left", "right"):
                    support_ratio = load.left_ratio if engine.support_leg == "left" else load.right_ratio
                    support_invalid_frames = support_invalid_frames + 1 if support_ratio < 0.42 else 0
                    if support_invalid_frames >= args.terrain_fsr_invalid_frames:
                        fault_reason = "SUPPORT FOOT LOST"
                        fault_latched = True
                        armed = False
                else:
                    support_invalid_frames = 0

                if fault_latched:
                    pose = dict(last_pose)
                    status = f"FAULT: {fault_reason}"
                else:
                    command = 0.0
                    step_elevation = 0.0
                    if armed and profile is not None:
                        controller.apply(engine, profile)
                        command = profile.command
                        step_elevation = profile.step_elevation_mm
                        status = profile.label
                    pose = engine.update(command, step_elevation_mm=step_elevation)

                    if (
                        armed
                        and load is not None
                        and abs(engine.last_step_elevation) > 0.05
                        and engine.last_phase_mode == "land"
                        and engine.last_landing_progress >= 0.78
                        and engine.last_swing_leg in ("left", "right")
                    ):
                        swing_load = load.left if engine.last_swing_leg == "left" else load.right
                        touchdown_invalid_frames = (
                            touchdown_invalid_frames + 1
                            if swing_load < args.terrain_touchdown_min_load
                            else 0
                        )
                        if touchdown_invalid_frames >= args.terrain_touchdown_invalid_frames:
                            fault_reason = f"NO {engine.last_swing_leg.upper()} FOOT CONTACT"
                            fault_latched = True
                            armed = False
                            pose = dict(last_pose)
                            status = f"FAULT: {fault_reason}"
                    else:
                        touchdown_invalid_frames = 0

                    if balance is not None and imu is not None:
                        now = time.monotonic()
                        dt = now - last_balance_t
                        last_balance_t = now
                        support_leg = engine.support_leg
                        if load is not None and load.total >= args.fsr_min_total_load:
                            if load.left_ratio >= args.fsr_support_ratio:
                                support_leg = "left"
                            elif load.right_ratio >= args.fsr_support_ratio:
                                support_leg = "right"
                        pose = balance.apply(
                            pose,
                            roll_deg=imu.roll_deg,
                            pitch_deg=imu.pitch_deg,
                            dt=dt,
                            support_leg=support_leg,
                        )

                backend.send(pose, duration_ms=args.update_ms)
                last_pose = dict(pose)

                if frame is not None and observation is not None:
                    preview = perception.draw(frame, observation)
                    preview = cv2.cvtColor(cv2.flip(preview, 1), cv2.COLOR_BGR2RGB)
                    surface = pygame.surfarray.make_surface(preview.swapaxes(0, 1))
                    screen.blit(surface, (0, 0))
                    state_label = "FAULT" if fault_latched else ("AUTO" if armed else "AUTO OFF")
                    confidence = round(observation.confidence * 100)
                    label = font.render(f"{state_label}  {status}", True, (245, 90, 90) if fault_latched else (70, 235, 165))
                    detail = small_font.render(
                        f"vision {confidence}%  IMU/FSR {'OK' if sensors_ok else 'WAIT'}",
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
                print(f"[terrain] Holding last pose because of fault: {fault_reason}. Support robot before power-off.")
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
