from __future__ import annotations

import argparse
import time

from src.balance import BalanceConfig, IMUBalanceController
from src.backends import SerialRTBackend
from src.config import Config, STANDING
from src.sensors import FootForceReading, RobotSensorHub, SensorSnapshot
from src.walking_engine import DynamicWalkingEngine, SingleSupportTestEngine


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Walking phase and one-foot balance debugger")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--sensor-port", default="/dev/ttyUSB0")
    parser.add_argument("--sensor-baudrate", type=int, default=115200)
    parser.add_argument("--step-command", type=float, default=0.22)
    parser.add_argument("--step-time", type=float, default=1.60)
    return parser.parse_args()


def open_sensors(args: argparse.Namespace) -> tuple[RobotSensorHub, IMUBalanceController]:
    settings = Config()
    sensor_hub = RobotSensorHub(
        port=args.sensor_port,
        baudrate=args.sensor_baudrate,
        timeout_s=settings.sensor_timeout_s,
        use_imu=True,
        use_hand_fsr=False,
        use_foot_fsr=True,
        imu_roll_sign=settings.imu_roll_sign,
        imu_pitch_sign=settings.imu_pitch_sign,
        imu_yaw_sign=settings.imu_yaw_sign,
        imu_vertical_mount=settings.imu_vertical_mount,
        imu_board_face_sign=settings.imu_board_face_sign,
        foot_fsr_invert=settings.foot_fsr_invert,
        foot_fsr_filter_alpha=settings.foot_fsr_filter_alpha,
        foot_fsr_zero_raw=settings.foot_fsr_zero_raw,
        foot_fsr_full_raw=settings.foot_fsr_full_raw,
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
    print(f"[walking-debug] Sensors ready: roll={roll:.2f}, pitch={pitch:.2f}.")
    return sensor_hub, balance


def make_walking_engine(args: argparse.Namespace) -> DynamicWalkingEngine:
    settings = Config()
    return DynamicWalkingEngine(
        dt=0.04,
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


def make_one_foot_engine() -> SingleSupportTestEngine:
    settings = Config()
    return SingleSupportTestEngine(
        dt=0.04,
        lift_height=settings.one_foot_lift_height,
        zmp_support_ratio=settings.zmp_support_ratio,
        hip_abduct_gain=settings.hip_abduct_gain,
        ankle_roll_gain=settings.ankle_roll_gain,
        arm_pwm=0,
        ramp_s=settings.one_foot_ramp_s,
    )


def phase_label(engine: DynamicWalkingEngine) -> str:
    labels = {
        "idle": "DOUBLE SUPPORT",
        "swing": "SWING: shift, raise, advance",
        "land": "LAND: lower and transfer",
    }
    return labels.get(engine.last_phase_mode, engine.last_phase_mode.upper())


def foot_text(feet: FootForceReading | None, leg: str) -> str:
    if feet is None:
        return "NO DATA"
    force = feet.left_force if leg == "left" else feet.right_force
    raw = feet.left_raw if leg == "left" else feet.right_raw
    return f"{force:.2f}  raw={raw if raw is not None else '-'}"


def foot_contact(feet: FootForceReading | None, leg: str, threshold: float) -> bool:
    if feet is None:
        return False
    force = feet.left_force if leg == "left" else feet.right_force
    return force >= threshold


def apply_imu(
    pose: dict[int, int],
    snapshot: SensorSnapshot | None,
    balance: IMUBalanceController | None,
    support_leg: str,
    last_balance_at: float,
) -> tuple[dict[int, int], float, str | None]:
    if balance is None:
        return pose, last_balance_at, None
    now = time.monotonic()
    reading = snapshot.imu if snapshot is not None else None
    settings = Config()
    if reading is None or not reading.balance_ready(settings.imu_min_gyro_cal, settings.imu_min_accel_cal):
        return pose, now, "IMU: WAITING"
    return balance.apply(pose, reading.roll_deg, reading.pitch_deg, now - last_balance_at, support_leg), now, None


def draw(
    screen,
    pygame,
    font,
    small_font,
    mode: str,
    engine: DynamicWalkingEngine,
    phase_running: bool,
    phase_step: bool,
    direction: int,
    support_leg: str,
    one_foot: SingleSupportTestEngine,
    snapshot: SensorSnapshot | None,
    one_foot_status: str,
    sensor_status: str,
) -> None:
    screen.fill((18, 22, 28))
    screen.blit(font.render("Walking Debug", True, (238, 242, 246)), (28, 18))
    screen.blit(small_font.render("1: Gait phases   2: One-foot balance", True, (184, 194, 204)), (28, 56))

    if mode == "phases":
        screen.blit(
            small_font.render("N: one phase  Space: continuous  F/B: direction  R: reset", True, (184, 194, 204)),
            (28, 82),
        )
        state = "RUNNING" if phase_running else "PAUSED"
        if phase_step:
            state += " - stop at boundary"
        rows = (
            ("Current phase", phase_label(engine)),
            ("Support leg", engine.support_leg),
            ("Swing leg", engine.last_swing_leg),
            ("Lift progress", f"{engine.last_lift_factor:.2f}"),
            ("Landing progress", f"{engine.last_landing_progress:.2f}"),
            ("Direction", "FORWARD" if direction > 0 else "BACKWARD"),
            ("Execution", state),
        )
    else:
        lifted_leg = "right" if support_leg == "left" else "left"
        screen.blit(
            small_font.render("L/R: support leg  Space: start/stop  C: standing", True, (184, 194, 204)),
            (28, 82),
        )
        rows = (
            ("Support leg", support_leg.upper()),
            ("Lifted leg", lifted_leg.upper()),
            ("Left FSR", foot_text(snapshot.feet if snapshot is not None else None, "left")),
            ("Right FSR", foot_text(snapshot.feet if snapshot is not None else None, "right")),
            ("Lift progress", f"{one_foot.phase:.2f}"),
            ("State", one_foot_status),
        )

    for index, (label, value) in enumerate(rows):
        y = 126 + index * 54
        screen.blit(small_font.render(label, True, (149, 161, 175)), (42, y))
        screen.blit(font.render(value, True, (72, 204, 166)), (42, y + 18))

    screen.blit(small_font.render("I: sensors on/off   Q/Esc: exit. Keep robot supported.", True, (245, 190, 72)), (28, 535))
    screen.blit(small_font.render(sensor_status, True, (58, 210, 148)), (28, 562))
    pygame.display.flip()


def main() -> None:
    args = parse_args()
    settings = Config()
    walking = make_walking_engine(args)
    one_foot = make_one_foot_engine()

    import pygame

    pygame.init()
    screen = pygame.display.set_mode((760, 610))
    pygame.display.set_caption("Humanoid Walking Debug")
    font = pygame.font.Font(None, 30)
    small_font = pygame.font.Font(None, 22)
    clock = pygame.time.Clock()
    mode = "phases"
    phase_running = False
    phase_step = False
    phase_started: str | None = None
    direction = 1
    selected_support = "right"
    phase_pose = dict(STANDING)
    sensor_hub = None
    balance = None
    snapshot = None
    last_balance_at = time.monotonic()
    last_sent_pose = None
    one_foot_status = "SENSORS OFF"
    sensor_status = "SENSORS: OFF (press I to connect IMU + foot FSR)"
    contact_frames = 0

    try:
        with SerialRTBackend(args.port, args.baudrate) as backend:
            backend.send(STANDING, duration_ms=1200, force=True)
            last_sent_pose = dict(STANDING)
            print(f"[walking-debug] Connected to {args.port}.")
            running = True
            while running:
                changed = False
                if sensor_hub is not None:
                    snapshot = sensor_hub.read()
                else:
                    snapshot = None
                feet = snapshot.feet if snapshot is not None else None
                both_contact = foot_contact(feet, "left", settings.foot_fsr_contact_threshold) and foot_contact(
                    feet, "right", settings.foot_fsr_contact_threshold
                )
                contact_frames = contact_frames + 1 if both_contact else 0

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key in (pygame.K_q, pygame.K_ESCAPE):
                            running = False
                        elif event.key == pygame.K_1:
                            mode = "phases"
                            one_foot.stop()
                            phase_running = False
                            phase_step = False
                            changed = True
                        elif event.key == pygame.K_2:
                            mode = "one-foot"
                            walking.reset()
                            phase_running = False
                            phase_step = False
                            one_foot.stop()
                            one_foot_status = "READY: select support L/R"
                            changed = True
                        elif event.key == pygame.K_i:
                            if sensor_hub is not None:
                                one_foot.stop()
                                sensor_hub.close()
                                sensor_hub = None
                                balance = None
                                one_foot_status = "SENSORS OFF"
                                sensor_status = "SENSORS: OFF"
                                changed = True
                            else:
                                try:
                                    sensor_hub, balance = open_sensors(args)
                                    last_balance_at = time.monotonic()
                                    sensor_status = "SENSORS: READY (IMU + 2 foot FSR)"
                                    one_foot_status = "READY: place both feet down"
                                except Exception as exc:
                                    sensor_status = f"SENSORS: OFF - {exc}"
                                    print(f"[walking-debug] {sensor_status}")
                        elif mode == "phases":
                            if event.key == pygame.K_n:
                                phase_running = True
                                phase_step = True
                                phase_started = None
                            elif event.key == pygame.K_SPACE:
                                phase_running = not phase_running
                                phase_step = False
                                phase_started = None
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
                        else:
                            if event.key == pygame.K_l:
                                selected_support = "left"
                            elif event.key == pygame.K_r:
                                selected_support = "right"
                            elif event.key == pygame.K_c:
                                one_foot.stop()
                                one_foot_status = "STANDING"
                                changed = True
                            elif event.key == pygame.K_SPACE:
                                if one_foot.running:
                                    one_foot.stop()
                                    one_foot_status = "STANDING"
                                    changed = True
                                elif balance is None or feet is None:
                                    one_foot_status = "WAIT: press I for IMU + FSR"
                                elif contact_frames < settings.foot_fsr_stable_frames:
                                    one_foot_status = "WAIT: both feet must contact floor"
                                else:
                                    one_foot.start(selected_support)
                                    one_foot_status = "LIFTING"

                base_pose = dict(STANDING)
                support_leg = "double"
                if mode == "phases" and phase_running:
                    phase_pose = walking.update(direction * abs(args.step_command))
                    base_pose = dict(phase_pose)
                    support_leg = walking.support_leg
                    current_phase = walking.last_phase_mode
                    if phase_step:
                        if phase_started is None and current_phase != "idle":
                            phase_started = current_phase
                        elif phase_started is not None and current_phase != phase_started:
                            phase_running = False
                            phase_step = False
                elif mode == "phases":
                    base_pose = dict(phase_pose)
                    support_leg = walking.support_leg
                elif one_foot.running:
                    support_leg = selected_support
                    if not foot_contact(feet, selected_support, settings.foot_fsr_contact_threshold):
                        one_foot.stop()
                        one_foot_status = "FAULT: SUPPORT FSR LOST - STANDING"
                        changed = True
                    else:
                        base_pose = one_foot.update()
                        lifted_leg = "right" if selected_support == "left" else "left"
                        if one_foot.phase >= 1.0 and foot_contact(
                            feet,
                            lifted_leg,
                            settings.foot_fsr_contact_threshold,
                        ):
                            one_foot_status = "LIFT BLOCKED: SWING FSR CONTACT"
                        else:
                            one_foot_status = "HOLDING" if one_foot.phase >= 1.0 else "LIFTING"

                command_pose, last_balance_at, imu_message = apply_imu(
                    base_pose,
                    snapshot,
                    balance,
                    support_leg,
                    last_balance_at,
                )
                if imu_message is not None:
                    sensor_status = imu_message

                moving = phase_running or one_foot.running
                if changed or command_pose != last_sent_pose:
                    backend.send(command_pose, duration_ms=80 if moving else 350, force=True)
                    last_sent_pose = command_pose

                draw(
                    screen,
                    pygame,
                    font,
                    small_font,
                    mode,
                    walking,
                    phase_running,
                    phase_step,
                    direction,
                    selected_support,
                    one_foot,
                    snapshot,
                    one_foot_status,
                    sensor_status,
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
