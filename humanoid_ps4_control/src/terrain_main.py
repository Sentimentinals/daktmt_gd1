from __future__ import annotations

import time

from .backends import make_backend
from .balance import (
    BalanceConfig,
    FallConfig,
    FallDetector,
    IMUBalanceController,
    angle_error_deg,
    extend_arms_forward,
)
from .config import Config, STANDING
from .sensors import RobotSensorHub


def _capture_reference(sensor_hub: RobotSensorHub, args: Config) -> tuple[float, float] | None:
    print("[terrain] Keep the robot upright and still on flat ground while IMU calibrates.")
    reference = sensor_hub.capture_imu_reference(
        sample_seconds=args.imu_reference_seconds,
        timeout_s=args.imu_reference_timeout_s,
        min_gyro_cal=args.imu_min_gyro_cal,
        min_accel_cal=args.imu_min_accel_cal,
        max_rms_deg=args.imu_reference_max_rms_deg,
    )
    if reference is None:
        print("[terrain] IMU reference failed. Balance remains OFF.")
    return reference


def _make_balance(reference: tuple[float, float], args: Config) -> IMUBalanceController:
    return IMUBalanceController(
        BalanceConfig(
            target_roll_deg=reference[0],
            target_pitch_deg=reference[1],
            max_correction_deg=args.terrain_balance_limit_deg,
            roll_deadband_deg=args.terrain_balance_deadband_deg,
            pitch_deadband_deg=args.terrain_balance_deadband_deg,
            pitch_ankle_gain=1.0,
            pitch_hip_gain=0.35,
            roll_ankle_gain=1.0,
            roll_hip_gain=0.30,
            double_support_gain=1.0,
        )
    )


def _draw_dashboard(screen, pygame, title_font, font, small_font, state: dict[str, object]) -> None:
    screen.fill((18, 22, 28))
    screen.blit(title_font.render("Terrain IMU Balance", True, (238, 242, 246)), (36, 26))

    enabled = bool(state["enabled"])
    ready = bool(state["ready"])
    fault = str(state["fault"])
    if fault:
        mode_text = f"FAULT: {fault}"
        mode_color = (245, 100, 100)
    elif enabled and ready:
        mode_text = "BALANCE ON"
        mode_color = (90, 225, 170)
    elif enabled:
        mode_text = "IMU WAIT"
        mode_color = (245, 190, 90)
    else:
        mode_text = "BALANCE OFF"
        mode_color = (170, 180, 192)
    screen.blit(font.render(mode_text, True, mode_color), (38, 78))

    rows = (
        ("Roll", f"{state['roll']:+7.2f} deg", f"target {state['target_roll']:+.2f}", f"error {state['roll_error']:+.2f}"),
        ("Pitch", f"{state['pitch']:+7.2f} deg", f"target {state['target_pitch']:+.2f}", f"error {state['pitch_error']:+.2f}"),
        ("Calibration", str(state["calibration"]), "S / G / A / M", ""),
        ("Ankle roll", str(state["ankle_roll"]), "PWM L / R", ""),
        ("Ankle pitch", str(state["ankle_pitch"]), "PWM L / R", ""),
        ("Hip roll", str(state["hip_roll"]), "PWM L / R", ""),
        ("Hip pitch", str(state["hip_pitch"]), "PWM L / R", ""),
    )
    for index, (label, value, detail, extra) in enumerate(rows):
        y = 128 + index * 46
        screen.blit(small_font.render(label, True, (150, 163, 178)), (42, y))
        screen.blit(font.render(value, True, (238, 242, 246)), (190, y - 5))
        screen.blit(small_font.render(detail, True, (150, 163, 178)), (400, y))
        if extra:
            screen.blit(small_font.render(extra, True, (150, 163, 178)), (540, y))
    pygame.display.flip()


def run_terrain(args: Config) -> None:
    try:
        import pygame
    except ImportError as exc:
        raise RuntimeError("Terrain IMU Balance requires pygame.") from exc

    from .keyboard_input import KeyboardReader

    control_hz = max(10, round(1000.0 / args.update_ms))
    reader = KeyboardReader(poll_rate_hz=control_hz)
    reader.init()
    screen = pygame.display.set_mode((700, 490))
    pygame.display.set_caption("Terrain IMU Balance")
    title_font = pygame.font.Font(None, 40)
    font = pygame.font.Font(None, 28)
    small_font = pygame.font.Font(None, 21)

    sensor_hub = RobotSensorHub(
        port=args.sensor_port,
        baudrate=args.sensor_baudrate,
        timeout_s=args.sensor_timeout_s,
        use_imu=True,
        use_foot_fsr=False,
        use_depth=False,
        imu_roll_sign=args.imu_roll_sign,
        imu_pitch_sign=args.imu_pitch_sign,
        imu_yaw_sign=args.imu_yaw_sign,
        imu_vertical_mount=args.imu_vertical_mount,
        imu_board_face_sign=args.imu_board_face_sign,
    )
    sensor_hub.open()
    backend = make_backend(mode=args.backend, port=args.port, baudrate=args.baudrate, csv_path=args.csv)

    reference: tuple[float, float] | None = None
    balance: IMUBalanceController | None = None
    fall_detector: FallDetector | None = None
    enabled = False
    fault = ""
    previous_toggle = False
    previous_stop = False
    previous_reset = False
    last_balance_at = time.monotonic()
    last_pose = dict(STANDING)

    try:
        with backend:
            backend.send(STANDING, duration_ms=900, force=True)
            time.sleep(1.0)
            reference = _capture_reference(sensor_hub, args)
            if reference is not None:
                balance = _make_balance(reference, args)
                if args.fall_detection_enabled:
                    fall_detector = FallDetector(
                        FallConfig(
                            trigger_tilt_deg=args.fall_trigger_tilt_deg,
                            trigger_rate_deg_s=args.fall_trigger_rate_deg_s,
                            hard_tilt_deg=args.fall_hard_tilt_deg,
                            consecutive_frames=args.fall_trigger_frames,
                        )
                    )
                enabled = True
                print("[terrain] IMU balance ON. V toggles, E/T recalibrates, C stops, O/Escape exits.")

            for keyboard in reader.poll():
                if keyboard.quit or keyboard.menu:
                    break

                toggle = keyboard.auto_toggle
                if toggle and not previous_toggle:
                    if fault:
                        print("[terrain] Clear the fault with E/T while the robot is upright on flat ground.")
                    elif balance is None:
                        print("[terrain] IMU reference unavailable. Press E/T to recalibrate.")
                    else:
                        enabled = not enabled
                        balance.reset()
                        print(f"[terrain] IMU balance {'ON' if enabled else 'OFF'}.")
                previous_toggle = toggle

                stop = keyboard.stop
                if stop and not previous_stop:
                    if fall_detector is not None and fall_detector.triggered:
                        print("[terrain] FALL latched. Holding protective pose; use E/T only when safe.")
                        last_pose = extend_arms_forward(
                            last_pose,
                            args.fall_arm_forward_pwm,
                        )
                        backend.send(last_pose, duration_ms=args.stop_ms, force=True)
                        previous_stop = stop
                        continue
                    enabled = False
                    fault = ""
                    if balance is not None:
                        balance.reset()
                    backend.send(STANDING, duration_ms=args.stop_ms, force=True)
                    print("[terrain] Balance OFF. Holding STANDING.")
                previous_stop = stop

                reset = keyboard.reset
                if reset and not previous_reset:
                    if fall_detector is not None and fall_detector.triggered:
                        snapshot = sensor_hub.read()
                        imu = snapshot.imu
                        reset_tilt = None
                        if imu is not None and reference is not None and imu.balance_ready(
                            args.imu_min_gyro_cal,
                            args.imu_min_accel_cal,
                        ):
                            reset_tilt = max(
                                abs(angle_error_deg(imu.roll_deg, reference[0])),
                                abs(angle_error_deg(imu.pitch_deg, reference[1])),
                            )
                        if reset_tilt is None or reset_tilt > args.fall_reset_tilt_deg:
                            print("[terrain] FALL reset blocked. Hold the robot upright, then press E/T again.")
                            previous_reset = reset
                            continue
                    enabled = False
                    fault = ""
                    backend.send(STANDING, duration_ms=700, force=True)
                    time.sleep(0.7)
                    reference = _capture_reference(sensor_hub, args)
                    balance = _make_balance(reference, args) if reference is not None else None
                    fall_detector = (
                        FallDetector(
                            FallConfig(
                                trigger_tilt_deg=args.fall_trigger_tilt_deg,
                                trigger_rate_deg_s=args.fall_trigger_rate_deg_s,
                                hard_tilt_deg=args.fall_hard_tilt_deg,
                                consecutive_frames=args.fall_trigger_frames,
                            )
                        )
                        if balance is not None and args.fall_detection_enabled
                        else None
                    )
                    enabled = balance is not None
                    last_balance_at = time.monotonic()
                previous_reset = reset

                snapshot = sensor_hub.read()
                imu = snapshot.imu
                ready = bool(
                    imu is not None
                    and imu.balance_ready(args.imu_min_gyro_cal, args.imu_min_accel_cal)
                )
                target_roll = reference[0] if reference is not None else 0.0
                target_pitch = reference[1] if reference is not None else 0.0
                roll = imu.roll_deg if imu is not None else 0.0
                pitch = imu.pitch_deg if imu is not None else 0.0
                roll_error = angle_error_deg(roll, target_roll) if imu is not None else 0.0
                pitch_error = angle_error_deg(pitch, target_pitch) if imu is not None else 0.0
                pose = dict(STANDING)

                now = time.monotonic()
                dt = now - last_balance_at
                last_balance_at = now
                fall_active = False
                if ready and balance is not None and fall_detector is not None:
                    fall_active = fall_detector.update(-roll_error, -pitch_error, dt)
                    if fall_active:
                        enabled = False
                        fault = f"FALL: {fall_detector.reason}"
                        pose = extend_arms_forward(
                            last_pose,
                            args.fall_arm_forward_pwm,
                        )
                if enabled and ready and balance is not None and not fall_active:
                    max_tilt = max(abs(roll_error), abs(pitch_error))
                    if max_tilt >= args.terrain_emergency_tilt_deg:
                        fault = f"TILT {max_tilt:.1f} DEG"
                        enabled = False
                        balance.reset()
                        print(f"[terrain] {fault}. Returning to STANDING.")
                    else:
                        pose = balance.apply(
                            pose,
                            roll_deg=roll,
                            pitch_deg=pitch,
                            dt=dt,
                            support_leg="double",
                        )

                backend.send(pose, duration_ms=args.update_ms)
                last_pose = dict(pose)
                calibration = (
                    f"{imu.system_cal} / {imu.gyro_cal} / {imu.accel_cal} / {imu.mag_cal}"
                    if imu is not None
                    else "-- / -- / -- / --"
                )
                _draw_dashboard(
                    screen,
                    pygame,
                    title_font,
                    font,
                    small_font,
                    {
                        "enabled": enabled,
                        "ready": ready,
                        "fault": fault,
                        "roll": roll,
                        "pitch": pitch,
                        "target_roll": target_roll,
                        "target_pitch": target_pitch,
                        "roll_error": roll_error,
                        "pitch_error": pitch_error,
                        "calibration": calibration,
                        "ankle_roll": f"{pose[16] - STANDING[16]:+d} / {pose[17] - STANDING[17]:+d}",
                        "ankle_pitch": f"{pose[15] - STANDING[15]:+d} / {pose[18] - STANDING[18]:+d}",
                        "hip_roll": f"{pose[12] - STANDING[12]:+d} / {pose[21] - STANDING[21]:+d}",
                        "hip_pitch": f"{pose[13] - STANDING[13]:+d} / {pose[20] - STANDING[20]:+d}",
                    },
                )

            exit_pose = last_pose if fall_detector is not None and fall_detector.triggered else STANDING
            backend.send(exit_pose, duration_ms=args.stop_ms, force=True)
            time.sleep(args.stop_ms / 1000.0)
    except KeyboardInterrupt:
        print("\n[terrain] Interrupted. Stopping control output.")
    finally:
        sensor_hub.close()
        reader.quit()
