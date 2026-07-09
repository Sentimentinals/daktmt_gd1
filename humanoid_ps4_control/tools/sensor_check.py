from __future__ import annotations

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.config import Config
from src.sensors import RobotSensorHub


def fmt(value: float | None, width: int = 7, digits: int = 3) -> str:
    if value is None:
        return " " * (width - 3) + "n/a"
    return f"{value:{width}.{digits}f}"


def main() -> None:
    cfg = Config()
    hub = RobotSensorHub(
        transport=cfg.sensor_transport,
        serial_port=cfg.sensor_port,
        serial_baudrate=cfg.sensor_baudrate,
        serial_timeout_s=cfg.sensor_timeout_s,
        use_imu=cfg.sensor_use_imu,
        use_fsr=cfg.sensor_use_fsr,
        imu_roll_sign=cfg.imu_roll_sign,
        imu_pitch_sign=cfg.imu_pitch_sign,
        imu_yaw_sign=cfg.imu_yaw_sign,
        fsr_ads1115_address=cfg.fsr_ads1115_address,
        fsr_left_channel=cfg.fsr_left_channel,
        fsr_right_channel=cfg.fsr_right_channel,
        fsr_invert=cfg.fsr_invert,
        fsr_filter_alpha=cfg.fsr_filter_alpha,
    )

    print("=== SENSOR CHECK ===")
    print(f"IMU enabled: {cfg.sensor_use_imu}")
    print(
        "FSR enabled: "
        f"{cfg.sensor_use_fsr}, ADS1115=0x{cfg.fsr_ads1115_address:02x}, "
        f"L=A{cfg.fsr_left_channel}, R=A{cfg.fsr_right_channel}, invert={cfg.fsr_invert}"
    )
    print("Press Ctrl+C to stop.\n")

    try:
        hub.open()
    except Exception as exc:
        print(f"[ERROR] Sensor open failed: {exc}")
        print("Check I2C wiring, i2cdetect -y 1, and required Python packages.")
        raise SystemExit(1) from exc

    period = max(0.05, cfg.update_ms / 1000.0)
    try:
        while True:
            snapshot = hub.read()
            imu = snapshot.imu
            foot = snapshot.foot_load

            roll = imu.roll_deg if imu is not None else None
            pitch = imu.pitch_deg if imu is not None else None
            yaw = imu.yaw_deg if imu is not None else None

            if foot is None:
                fsr_text = "FSR n/a"
            else:
                fsr_text = (
                    f"FSR L={foot.left:.3f} ({foot.left_voltage:.3f}V) "
                    f"R={foot.right:.3f} ({foot.right_voltage:.3f}V) "
                    f"ratio L={foot.left_ratio:.2f} R={foot.right_ratio:.2f}"
                )

            print(
                f"IMU roll={fmt(roll)} pitch={fmt(pitch)} yaw={fmt(yaw)} | {fsr_text}"
            )
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nSensor check stopped.")
    finally:
        hub.close()


if __name__ == "__main__":
    main()
