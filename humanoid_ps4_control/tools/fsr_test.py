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


def format_reading(reading: FootLoadReading, raw_left: int | None = None, raw_right: int | None = None) -> str:
    raw_text = ""
    if raw_left is not None and raw_right is not None:
        raw_text = f" raw L={raw_left:4d} R={raw_right:4d}"
    return (
        f"L={reading.left:.3f} ({reading.left_voltage:.3f}V) "
        f"R={reading.right:.3f} ({reading.right_voltage:.3f}V) "
        f"ratio L={reading.left_ratio:.2f} R={reading.right_ratio:.2f} "
        f"total={reading.total:.3f}{raw_text}"
    )


def parse_serial_fsr_line(line: str) -> tuple[FootLoadReading, int | None, int | None] | None:
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
        raw_left = data.get("left_raw", data.get("lr"))
        raw_right = data.get("right_raw", data.get("rr"))
        return (
            FootLoadReading(
                left=left,
                right=right,
                left_voltage=float(data.get("left_voltage", data.get("lv", 0.0))),
                right_voltage=float(data.get("right_voltage", data.get("rv", 0.0))),
            ),
            int(raw_left) if raw_left is not None else None,
            int(raw_right) if raw_right is not None else None,
        )

    if not line.startswith("F,"):
        return None

    parts = [part.strip() for part in line.split(",")]
    values = parts[1:]
    if len(values) in {3, 5, 7}:
        values = values[1:]
    if len(values) < 2:
        return None

    left = float(values[0])
    right = float(values[1])
    left_voltage = float(values[2]) if len(values) >= 3 else 0.0
    right_voltage = float(values[3]) if len(values) >= 4 else 0.0
    raw_left = int(values[4]) if len(values) >= 5 else None
    raw_right = int(values[5]) if len(values) >= 6 else None
    return (
        FootLoadReading(
            left=left,
            right=right,
            left_voltage=left_voltage,
            right_voltage=right_voltage,
        ),
        raw_left,
        raw_right,
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
    print("If this stays at 'waiting', flash the ESP32 firmware that prints FSR packets.")
    print("Press Ctrl+C to stop.\n")

    with serial.Serial(args.port, args.baudrate, timeout=1.0) as ser:
        last_packet_t = time.monotonic()
        line_count = 0
        try:
            while True:
                raw = ser.readline().decode("utf-8", errors="replace")
                line_count += 1 if raw else 0
                try:
                    parsed = parse_serial_fsr_line(raw)
                except (TypeError, ValueError):
                    parsed = None
                if parsed is not None:
                    last_packet_t = time.monotonic()
                    reading, raw_left, raw_right = parsed
                    print(format_reading(reading, raw_left, raw_right))
                elif time.monotonic() - last_packet_t >= args.no_packet_warning_s:
                    print(f"waiting for FSR packet... serial lines seen={line_count}")
                    last_packet_t = time.monotonic()
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
    parser.add_argument("--no-packet-warning-s", type=float, default=3.0)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.mode == "serial":
        run_serial(args)
    else:
        run_ads1115(args)


if __name__ == "__main__":
    main()
