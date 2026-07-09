from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.config import Config
from src.sensors import ADS1115FootLoadReader, FootLoadReading


def format_reading(reading: FootLoadReading) -> str:
    return (
        f"L={reading.left:.3f} ({reading.left_voltage:.3f}V) "
        f"R={reading.right:.3f} ({reading.right_voltage:.3f}V) "
        f"ratio L={reading.left_ratio:.2f} R={reading.right_ratio:.2f} "
        f"total={reading.total:.3f}"
    )


def parse_serial_fsr_line(line: str) -> FootLoadReading | None:
    line = line.strip()
    if not line:
        return None

    if line.startswith("{"):
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            return None
        if data.get("type") not in {"fsr", "F"}:
            return None
        left = float(data["left"])
        right = float(data["right"])
        return FootLoadReading(
            left=left,
            right=right,
            left_voltage=float(data.get("left_voltage", data.get("lv", 0.0))),
            right_voltage=float(data.get("right_voltage", data.get("rv", 0.0))),
        )

    if not line.startswith("F,"):
        return None

    parts = [part.strip() for part in line.split(",")]
    values = parts[1:]
    if len(values) >= 3:
        values = values[1:]
    if len(values) < 2:
        return None

    left = float(values[0])
    right = float(values[1])
    left_voltage = float(values[2]) if len(values) >= 3 else 0.0
    right_voltage = float(values[3]) if len(values) >= 4 else 0.0
    return FootLoadReading(
        left=left,
        right=right,
        left_voltage=left_voltage,
        right_voltage=right_voltage,
    )


def run_ads1115(args: argparse.Namespace) -> None:
    reader = ADS1115FootLoadReader(
        address=args.address,
        left_channel=args.left_channel,
        right_channel=args.right_channel,
        invert=args.invert,
        filter_alpha=args.filter_alpha,
    )
    reader.open()
    print(
        "FSR ADS1115 test: "
        f"address=0x{args.address:02x}, L=A{args.left_channel}, R=A{args.right_channel}, "
        f"invert={args.invert}"
    )
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            print(format_reading(reader.read()))
            time.sleep(args.period)
    except KeyboardInterrupt:
        print("\nFSR test stopped.")


def run_serial(args: argparse.Namespace) -> None:
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("Serial mode requires: pip install pyserial") from exc

    print(f"FSR serial test: port={args.port}, baudrate={args.baudrate}")
    print("Accepts: F,left,right[,left_voltage,right_voltage] or F,ms,left,right[,left_voltage,right_voltage]")
    print("Press Ctrl+C to stop.\n")

    with serial.Serial(args.port, args.baudrate, timeout=1.0) as ser:
        try:
            while True:
                raw = ser.readline().decode("utf-8", errors="replace")
                try:
                    reading = parse_serial_fsr_line(raw)
                except (TypeError, ValueError):
                    reading = None
                if reading is not None:
                    print(format_reading(reading))
        except KeyboardInterrupt:
            print("\nFSR serial test stopped.")


def build_parser() -> argparse.ArgumentParser:
    cfg = Config()
    parser = argparse.ArgumentParser(description="Test left/right FSR readings.")
    parser.add_argument(
        "--mode",
        choices=("ads1115", "serial"),
        default="ads1115",
        help="ads1115 reads local I2C ADC; serial reads FSR packets from ESP32/Pico USB.",
    )
    parser.add_argument("--period", type=float, default=max(0.05, cfg.update_ms / 1000.0))
    parser.add_argument("--address", type=lambda value: int(value, 0), default=cfg.fsr_ads1115_address)
    parser.add_argument("--left-channel", type=int, default=cfg.fsr_left_channel)
    parser.add_argument("--right-channel", type=int, default=cfg.fsr_right_channel)
    parser.add_argument("--invert", action="store_true", default=cfg.fsr_invert)
    parser.add_argument("--filter-alpha", type=float, default=cfg.fsr_filter_alpha)
    parser.add_argument("--port", default=cfg.sensor_port)
    parser.add_argument("--baudrate", type=int, default=cfg.sensor_baudrate)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.mode == "serial":
        run_serial(args)
    else:
        run_ads1115(args)


if __name__ == "__main__":
    main()
