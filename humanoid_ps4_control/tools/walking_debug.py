from __future__ import annotations

import argparse
import time

from src.balance import BalanceConfig, IMUBalanceController
from src.backends import SerialRTBackend
from src.config import Config, STANDING
from src.sensors import RobotSensorHub
from src.walking_engine import DynamicWalkingEngine


LEGS = (
    (12, "Left hip roll"),
    (13, "Left hip pitch"),
    (14, "Left knee"),
    (15, "Left ankle pitch"),
    (16, "Left ankle roll"),
    (17, "Right ankle roll"),
    (18, "Right ankle pitch"),
    (19, "Right knee"),
    (20, "Right hip pitch"),
    (21, "Right hip roll"),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Walking phase and leg calibration debugger")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--sensor-port", default="/dev/ttyUSB0")
    parser.add_argument("--sensor-baudrate", type=int, default=115200)
    parser.add_argument("--min-pwm", type=int, default=1350)
    parser.add_argument("--max-pwm", type=int, default=1650)
    parser.add_argument("--step-command", type=float, default=0.22)
    parser.add_argument("--step-time", type=float, default=1.60)
    return parser.parse_args()


def print_pose(pose: dict[int, int]) -> None:
    values = ", ".join(f"{servo_id}: {pose[servo_id]}" for servo_id, _ in LEGS)
    print(f"LEG_STANDING = {{{values}}}")


def open_imu(args: argparse.Namespace):
    settings = Config()
    sensor_hub = RobotSensorHub(
        port=args.sensor_port,
        baudrate=args.sensor_baudrate,
        timeout_s=settings.sensor_timeout_s,
        use_imu=True,
        use_hand_fsr=False,
        imu_roll_sign=settings.imu_roll_sign,
        imu_pitch_sign=settings.imu_pitch_sign,
        imu_yaw_sign=settings.imu_yaw_sign,
        imu_vertical_mount=settings.imu_vertical_mount,
        imu_board_face_sign=settings.imu_board_face_sign,
    )
    sensor_hub.open()
    print("[walking-debug] Keep robot still while IMU reference is captured.")
    reference = sensor_hub.capture_imu_reference(
        sample_seconds=settings.imu_reference_seconds,
        timeout_s=settings.imu_reference_timeout_s,
        min_gyro_cal=settings.imu_min_gyro_cal,
        min_accel_cal=settings.imu_min_accel_cal,
        max_rms_deg=settings.imu_reference_max_rms_deg,
    )
    if reference is None:
        sensor_hub.close()
        raise RuntimeError("IMU reference failed. Keep the robot still and retry.")
    roll, pitch = reference
    balance = IMUBalanceController(
        BalanceConfig(
            target_roll_deg=roll,
            target_pitch_deg=pitch,
            max_correction_deg=settings.balance_limit_deg,
            roll_deadband_deg=settings.balance_deadband_deg,
            pitch_deadband_deg=settings.balance_deadband_deg,
        )
    )
    print(f"[walking-debug] IMU ON: reference roll={roll:.2f}, pitch={pitch:.2f}.")
    return sensor_hub, balance


def make_walking_engine(args: argparse.Namespace) -> DynamicWalkingEngine:
    settings = Config()
    dt = 0.04
    return DynamicWalkingEngine(
        dt=dt,
        t_step=max(1.20, args.step_time),
        t_dbl=settings.t_dbl,
        max_step_len=settings.max_step_len,
        max_turn_step_len=settings.max_turn_step_len,
        max_side_step_len=settings.max_side_step_len,
        step_height=settings.step_height,
        zmp_support_ratio=settings.zmp_support_ratio,
        hip_abduct_gain=settings.hip_abduct_gain,
        swing_hip_roll_scale=settings.swing_hip_roll_scale,
        ankle_roll_gain=settings.ankle_roll_gain,
        swing_ankle_roll_scale=settings.swing_ankle_roll_scale,
        step_x_ratio=settings.step_x_ratio,
        thigh_lift_forward_mm=settings.thigh_lift_forward_mm,
        left_swing_x_scale=settings.left_swing_x_scale,
        left_step_height_scale=settings.left_step_height_scale,
        landing_gap_mm=settings.landing_gap_mm,
        right_swing_x_scale=settings.right_swing_x_scale,
        right_step_height_scale=settings.right_step_height_scale,
        lift_start_phase=settings.lift_start_phase,
        swing_advance_end_phase=settings.swing_advance_end_phase,
        lift_end_phase=settings.lift_end_phase,
        landing_roll_release_start=settings.landing_roll_release_start,
        command_rate_limit=settings.command_rate_limit,
        arm_swing_pwm=settings.arm_swing_pwm,
        arm_right_dir=settings.arm_right_dir,
        arm_left_dir=settings.arm_left_dir,
        arm_elbow_ratio=settings.arm_elbow_ratio,
        arm_lift_ratio=settings.arm_lift_ratio,
        arm_smooth_tau=settings.arm_smooth_tau,
        arm_min_pwm=settings.arm_min_pwm,
        arm_quantum_pwm=settings.arm_quantum_pwm,
        trajectory_smoothing=settings.trajectory_smoothing,
    )


def phase_label(engine: DynamicWalkingEngine) -> str:
    labels = {
        "idle": "DOUBLE SUPPORT",
        "swing": "SWING: shift weight, raise and advance foot",
        "land": "LAND: lower foot and transfer support",
    }
    return labels.get(engine.last_phase_mode, engine.last_phase_mode.upper())


def draw(
    screen,
    pygame,
    font,
    small_font,
    mode: str,
    selected: int,
    calibration_pose: dict[int, int],
    minimum: int,
    maximum: int,
    imu_status: str,
    engine: DynamicWalkingEngine,
    phase_running: bool,
    phase_step: bool,
    direction: int,
) -> None:
    screen.fill((18, 22, 28))
    title = font.render("Walking Phase Debug", True, (238, 242, 246))
    screen.blit(title, (28, 18))
    tabs = "1: Calibration   2: Gait phases"
    screen.blit(small_font.render(tabs, True, (184, 194, 204)), (28, 56))

    if mode == "calibration":
        help_text = "Up/Down select  Left/Right: 5us  Shift: 20us  Home: 1500  S: print  C: all 1500"
        screen.blit(small_font.render(help_text, True, (184, 194, 204)), (28, 82))
        for index, (servo_id, label) in enumerate(LEGS):
            y = 116 + index * 38
            active = index == selected
            rect = pygame.Rect(28, y, 620, 30)
            pygame.draw.rect(screen, (43, 104, 92) if active else (39, 45, 54), rect)
            pygame.draw.rect(screen, (100, 210, 174) if active else (75, 84, 96), rect, 2)
            text = f"{servo_id:02d}  {label:<20} {calibration_pose[servo_id]:4d} us"
            screen.blit(small_font.render(text, True, (248, 250, 252)), (42, y + 7))
    else:
        help_text = "N: one complete phase  Space: continuous  F/B: direction  R: reset standing"
        screen.blit(small_font.render(help_text, True, (184, 194, 204)), (28, 82))
        state = "RUNNING" if phase_running else "PAUSED"
        if phase_step:
            state += " - stop at next phase boundary"
        rows = (
            ("Current phase", phase_label(engine)),
            ("Support leg", engine.support_leg),
            ("Swing leg", engine.last_swing_leg),
            ("Lift progress", f"{engine.last_lift_factor:.2f}"),
            ("Landing progress", f"{engine.last_landing_progress:.2f}"),
            ("Direction", "FORWARD" if direction > 0 else "BACKWARD"),
            ("Execution", state),
        )
        for index, (label, value) in enumerate(rows):
            y = 126 + index * 48
            screen.blit(small_font.render(label, True, (149, 161, 175)), (42, y))
            screen.blit(font.render(value, True, (72, 204, 166)), (42, y + 16))

    footer = "I: IMU on/off   Q/Esc: exit. Keep robot supported during phase debugging."
    screen.blit(small_font.render(footer, True, (245, 190, 72)), (28, 535))
    screen.blit(small_font.render(imu_status, True, (58, 210, 148)), (28, 562))
    pygame.display.flip()


def apply_imu(
    pose: dict[int, int],
    sensor_hub: RobotSensorHub | None,
    balance: IMUBalanceController | None,
    support_leg: str,
    last_balance_at: float,
    minimum: int,
    maximum: int,
) -> tuple[dict[int, int], float, str | None]:
    if sensor_hub is None or balance is None:
        return pose, last_balance_at, None
    now = time.monotonic()
    reading = sensor_hub.read().imu
    if reading is None or not reading.balance_ready(Config().imu_min_gyro_cal, Config().imu_min_accel_cal):
        return pose, now, "IMU: ON - waiting for valid reading"
    corrected = balance.apply(
        pose,
        roll_deg=reading.roll_deg,
        pitch_deg=reading.pitch_deg,
        dt=now - last_balance_at,
        support_leg=support_leg,
    )
    corrected = {servo_id: max(minimum, min(maximum, value)) for servo_id, value in corrected.items()}
    return corrected, now, None


def main() -> None:
    args = parse_args()
    minimum = min(args.min_pwm, args.max_pwm)
    maximum = max(args.min_pwm, args.max_pwm)
    calibration_pose = {servo_id: max(minimum, min(maximum, STANDING[servo_id])) for servo_id, _ in LEGS}
    walking = make_walking_engine(args)

    import pygame

    pygame.init()
    screen = pygame.display.set_mode((760, 610))
    pygame.display.set_caption("Humanoid Walking Phase Debug")
    font = pygame.font.Font(None, 30)
    small_font = pygame.font.Font(None, 22)
    clock = pygame.time.Clock()
    selected = 0
    mode = "calibration"
    phase_running = False
    phase_step = False
    phase_started: str | None = None
    direction = 1
    phase_pose = dict(STANDING)
    sensor_hub = None
    balance = None
    last_balance_at = time.monotonic()
    last_sent_pose = None
    imu_status = "IMU: OFF (press I to compare without/with balance)"

    try:
        with SerialRTBackend(args.port, args.baudrate) as backend:
            backend.send(calibration_pose, duration_ms=1200, force=True)
            last_sent_pose = dict(calibration_pose)
            print(f"[walking-debug] Connected to {args.port}.")
            print("[walking-debug] Use mode 2, then N to run one gait phase at a time.")
            print_pose(calibration_pose)
            running = True
            while running:
                changed = False
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key in (pygame.K_q, pygame.K_ESCAPE):
                            running = False
                        elif event.key == pygame.K_1:
                            mode = "calibration"
                            phase_running = False
                            phase_step = False
                            changed = True
                        elif event.key == pygame.K_2:
                            mode = "phases"
                            phase_running = False
                            phase_step = False
                            walking.reset()
                            phase_pose = dict(STANDING)
                            changed = True
                            print("[walking-debug] Phase debugger reset: DOUBLE SUPPORT.")
                        elif event.key == pygame.K_i:
                            if sensor_hub is not None:
                                sensor_hub.close()
                                sensor_hub = None
                                balance = None
                                imu_status = "IMU: OFF (press I to compare without/with balance)"
                                print("[walking-debug] IMU OFF.")
                            else:
                                try:
                                    sensor_hub, balance = open_imu(args)
                                    last_balance_at = time.monotonic()
                                    imu_status = "IMU: ON - balance correction active"
                                except Exception as exc:
                                    imu_status = f"IMU: OFF - {exc}"
                                    print(f"[walking-debug] {imu_status}")
                        elif mode == "calibration":
                            if event.key == pygame.K_UP:
                                selected = (selected - 1) % len(LEGS)
                            elif event.key == pygame.K_DOWN:
                                selected = (selected + 1) % len(LEGS)
                            elif event.key in (pygame.K_LEFT, pygame.K_RIGHT):
                                step = 20 if event.mod & pygame.KMOD_SHIFT else 5
                                if event.key == pygame.K_LEFT:
                                    step = -step
                                servo_id = LEGS[selected][0]
                                calibration_pose[servo_id] = max(
                                    minimum, min(maximum, calibration_pose[servo_id] + step)
                                )
                                changed = True
                            elif event.key == pygame.K_HOME:
                                calibration_pose[LEGS[selected][0]] = 1500
                                changed = True
                            elif event.key == pygame.K_c:
                                for servo_id, _ in LEGS:
                                    calibration_pose[servo_id] = 1500
                                changed = True
                            elif event.key == pygame.K_s:
                                print_pose(calibration_pose)
                        elif mode == "phases":
                            if event.key == pygame.K_n:
                                phase_running = True
                                phase_step = True
                                phase_started = None
                                print("[walking-debug] Running one complete gait phase.")
                            elif event.key == pygame.K_SPACE:
                                phase_running = not phase_running
                                phase_step = False
                                phase_started = None
                                print(f"[walking-debug] Continuous gait {'ON' if phase_running else 'PAUSED'}.")
                            elif event.key == pygame.K_f:
                                direction = 1
                            elif event.key == pygame.K_b:
                                direction = -1
                            elif event.key == pygame.K_r:
                                walking.reset()
                                phase_running = False
                                phase_step = False
                                phase_started = None
                                phase_pose = dict(STANDING)
                                changed = True
                                print("[walking-debug] Gait reset to standing.")

                base_pose = dict(calibration_pose)
                support_leg = "double"
                if mode == "phases" and phase_running:
                    phase_pose = walking.update(direction * abs(args.step_command))
                    base_pose = dict(phase_pose)
                    support_leg = walking.support_leg
                    current_phase = walking.last_phase_mode
                    if phase_step:
                        if phase_started is None and current_phase != "idle":
                            phase_started = current_phase
                            print(f"[walking-debug] Entered {phase_label(walking)}.")
                        elif phase_started is not None and current_phase != phase_started:
                            phase_running = False
                            phase_step = False
                            print(f"[walking-debug] Paused at {phase_label(walking)}.")
                elif mode == "phases":
                    base_pose = dict(phase_pose)
                    support_leg = walking.support_leg

                command_pose, last_balance_at, imu_message = apply_imu(
                    base_pose,
                    sensor_hub,
                    balance,
                    support_leg,
                    last_balance_at,
                    minimum,
                    maximum,
                )
                if imu_message is not None:
                    imu_status = imu_message

                if changed or command_pose != last_sent_pose:
                    backend.send(command_pose, duration_ms=80 if phase_running else 250, force=True)
                    last_sent_pose = command_pose

                draw(
                    screen,
                    pygame,
                    font,
                    small_font,
                    mode,
                    selected,
                    calibration_pose,
                    minimum,
                    maximum,
                    imu_status,
                    walking,
                    phase_running,
                    phase_step,
                    direction,
                )
                clock.tick(25)
    except KeyboardInterrupt:
        pass
    finally:
        if sensor_hub is not None:
            sensor_hub.close()
        pygame.quit()
        print("[walking-debug] Closed.")


if __name__ == "__main__":
    main()
