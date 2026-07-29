from __future__ import annotations

import argparse
import time

from src.balance import BalanceConfig, IMUBalanceController
from src.backends import SerialRTBackend
from src.config import Config, STANDING
from src.sensors import RobotSensorHub


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
    parser = argparse.ArgumentParser(description="Interactive leg-servo calibration")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--sensor-port", default="/dev/ttyUSB0")
    parser.add_argument("--sensor-baudrate", type=int, default=115200)
    parser.add_argument("--min-pwm", type=int, default=1350)
    parser.add_argument("--max-pwm", type=int, default=1650)
    return parser.parse_args()


def draw(
    screen,
    pygame,
    font,
    small_font,
    selected: int,
    pose: dict[int, int],
    minimum: int,
    maximum: int,
    imu_status: str,
) -> None:
    screen.fill((18, 22, 28))
    title = font.render("Walking Leg Calibration", True, (238, 242, 246))
    screen.blit(title, (28, 22))
    help_text = (
        "Up/Down select   Left/Right: 5us   Shift+Left/Right: 20us   "
        "Home: 1500us   I: IMU on/off   S: print values   C: all 1500us   Q/Esc: exit"
    )
    screen.blit(small_font.render(help_text, True, (184, 194, 204)), (28, 64))
    for index, (servo_id, label) in enumerate(LEGS):
        y = 104 + index * 42
        active = index == selected
        rect = pygame.Rect(28, y, 584, 32)
        pygame.draw.rect(screen, (43, 104, 92) if active else (39, 45, 54), rect)
        pygame.draw.rect(screen, (100, 210, 174) if active else (75, 84, 96), rect, 2)
        text = f"{servo_id:02d}  {label:<20} {pose[servo_id]:4d} us"
        screen.blit(small_font.render(text, True, (248, 250, 252)), (42, y + 8))
    footer = f"Safe range: {minimum}-{maximum} us. Keep robot supported; this tool controls legs only."
    screen.blit(small_font.render(footer, True, (245, 190, 72)), (28, 548))
    screen.blit(small_font.render(imu_status, True, (58, 210, 148)), (28, 570))
    pygame.display.flip()


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


def main() -> None:
    args = parse_args()
    minimum = min(args.min_pwm, args.max_pwm)
    maximum = max(args.min_pwm, args.max_pwm)
    pose = {servo_id: max(minimum, min(maximum, STANDING[servo_id])) for servo_id, _ in LEGS}

    import pygame

    pygame.init()
    screen = pygame.display.set_mode((760, 620))
    pygame.display.set_caption("Humanoid Walking Calibration")
    font = pygame.font.Font(None, 36)
    small_font = pygame.font.Font(None, 24)
    clock = pygame.time.Clock()
    selected = 0
    sensor_hub = None
    balance = None
    last_balance_at = time.monotonic()
    last_sent_pose = None
    imu_status = "IMU: OFF (press I to enable balance test)"

    try:
        with SerialRTBackend(args.port, args.baudrate) as backend:
            backend.send(pose, duration_ms=1200, force=True)
            print(f"[walking-debug] Connected to {args.port}.")
            print_pose(pose)
            running = True
            while running:
                changed = False
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key in (pygame.K_q, pygame.K_ESCAPE):
                            running = False
                        elif event.key == pygame.K_UP:
                            selected = (selected - 1) % len(LEGS)
                        elif event.key == pygame.K_DOWN:
                            selected = (selected + 1) % len(LEGS)
                        elif event.key in (pygame.K_LEFT, pygame.K_RIGHT):
                            step = 20 if event.mod & pygame.KMOD_SHIFT else 5
                            if event.key == pygame.K_LEFT:
                                step = -step
                            servo_id = LEGS[selected][0]
                            pose[servo_id] = max(minimum, min(maximum, pose[servo_id] + step))
                            changed = True
                        elif event.key == pygame.K_HOME:
                            pose[LEGS[selected][0]] = 1500
                            changed = True
                        elif event.key == pygame.K_c:
                            for servo_id, _ in LEGS:
                                pose[servo_id] = 1500
                            changed = True
                        elif event.key == pygame.K_s:
                            print_pose(pose)
                        elif event.key == pygame.K_i:
                            if sensor_hub is not None:
                                sensor_hub.close()
                                sensor_hub = None
                                balance = None
                                imu_status = "IMU: OFF (press I to enable balance test)"
                                print("[walking-debug] IMU OFF.")
                            else:
                                try:
                                    sensor_hub, balance = open_imu(args)
                                    last_balance_at = time.monotonic()
                                    imu_status = "IMU: ON - balance correction active"
                                except Exception as exc:
                                    imu_status = f"IMU: OFF - {exc}"
                                    print(f"[walking-debug] {imu_status}")

                command_pose = dict(pose)
                if sensor_hub is not None and balance is not None:
                    now = time.monotonic()
                    reading = sensor_hub.read().imu
                    if reading is not None and reading.balance_ready(
                        Config().imu_min_gyro_cal,
                        Config().imu_min_accel_cal,
                    ):
                        command_pose = balance.apply(
                            command_pose,
                            roll_deg=reading.roll_deg,
                            pitch_deg=reading.pitch_deg,
                            dt=now - last_balance_at,
                        )
                        command_pose = {
                            servo_id: max(minimum, min(maximum, value))
                            for servo_id, value in command_pose.items()
                        }
                    else:
                        imu_status = "IMU: ON - waiting for valid reading"
                    last_balance_at = now

                if changed or command_pose != last_sent_pose:
                    backend.send(command_pose, duration_ms=180 if balance is not None else 350, force=True)
                    last_sent_pose = command_pose
                draw(screen, pygame, font, small_font, selected, pose, minimum, maximum, imu_status)
                clock.tick(30)
    except KeyboardInterrupt:
        pass
    finally:
        if sensor_hub is not None:
            sensor_hub.close()
        pygame.quit()
        print("[walking-debug] Closed. Copy LEG_STANDING from the terminal before changing config.")
        time.sleep(0.1)


if __name__ == "__main__":
    main()
