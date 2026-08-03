from __future__ import annotations

import argparse
import time

from src.config import Config
from src.sensors import RobotSensorHub


def parse_args() -> argparse.Namespace:
    settings = Config()
    parser = argparse.ArgumentParser(description="Print ESP32 IMU, FSR, and ToF data without moving servos")
    parser.add_argument("--port", default=settings.sensor_port)
    parser.add_argument("--baudrate", type=int, default=settings.sensor_baudrate)
    parser.add_argument("--seconds", type=float, default=15.0)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--imu-only", action="store_true")
    mode.add_argument("--fsr-only", action="store_true")
    mode.add_argument("--tof-only", action="store_true")
    mode.add_argument("--imu-tof", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    settings = Config()
    use_imu = not args.fsr_only and not args.tof_only
    use_fsr = not args.imu_only and not args.tof_only and not args.imu_tof
    use_depth = not args.imu_only and not args.fsr_only
    hub = RobotSensorHub(
        port=args.port,
        baudrate=args.baudrate,
        timeout_s=settings.sensor_timeout_s,
        depth_timeout_s=settings.sensor_depth_timeout_s,
        use_imu=use_imu,
        use_foot_fsr=use_fsr,
        use_depth=use_depth,
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
    hub.open()
    print(
        f"[sensor-monitor] Reading {hub.active_port or args.port} "
        f"at {args.baudrate} baud for {args.seconds:.0f}s."
    )
    deadline = time.monotonic() + max(1.0, args.seconds)
    next_print = 0.0
    imu_seen = False
    feet_seen = False
    depth_seen = False
    try:
        while time.monotonic() < deadline:
            snapshot = hub.read()
            imu_seen = imu_seen or snapshot.imu is not None
            feet_seen = feet_seen or snapshot.feet is not None
            depth_seen = depth_seen or snapshot.depth is not None
            if time.monotonic() < next_print:
                time.sleep(0.01)
                continue

            imu = snapshot.imu
            feet = snapshot.feet
            depth = snapshot.depth
            imu_text = (
                "IMU: disabled"
                if not use_imu
                else
                "IMU: waiting"
                if imu is None
                else (
                    f"IMU roll={imu.roll_deg:+6.2f} pitch={imu.pitch_deg:+6.2f} "
                    f"cal={imu.system_cal}/{imu.gyro_cal}/{imu.accel_cal}/{imu.mag_cal}"
                )
            )
            feet_text = (
                "FSR: disabled"
                if not use_fsr
                else
                "FSR: waiting"
                if feet is None
                else (
                    f"FSR L={feet.left_force:.2f} raw={feet.left_raw} "
                    f"R={feet.right_force:.2f} raw={feet.right_raw}"
                )
            )
            depth_text = (
                "ToF: disabled"
                if not use_depth
                else "ToF: waiting"
                if depth is None
                else (
                    f"ToF center={depth.center_distance_mm}mm "
                    f"object={depth.obstacle_distance_mm}mm span={depth.vertical_span_mm}mm"
                )
            )
            print(f"{imu_text} | {feet_text} | {depth_text}")
            next_print = time.monotonic() + 0.10
    finally:
        hub.close()

    passed = (not use_imu or imu_seen) and (not use_fsr or feet_seen) and (not use_depth or depth_seen)
    if passed:
        if args.imu_only:
            label = "IMU"
        elif args.fsr_only:
            label = "foot FSR"
        elif args.tof_only:
            label = "VL53L5CX"
        elif args.imu_tof:
            label = "IMU and VL53L5CX"
        else:
            label = "IMU, foot FSR, and VL53L5CX"
        print(f"[sensor-monitor] PASS: {label} stream was received.")
        return 0
    label = "requested sensor"
    print(f"[sensor-monitor] FAIL: missing {label} stream. Check ESP32 USB port and firmware.")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
