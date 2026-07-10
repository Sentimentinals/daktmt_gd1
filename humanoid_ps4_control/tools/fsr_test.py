from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.config import Config
from src.sensors import FootLoadReading


def parse_fsr_packet(line: str) -> tuple[FootLoadReading, int | None, int | None] | None:
    fields = [field.strip() for field in line.strip().split(",")]
    if not fields or fields[0] != "F":
        return None

    values = fields[1:]
    if len(values) in {3, 5, 7}:
        values = values[1:]
    if len(values) < 2:
        return None

    left = float(values[0])
    right = float(values[1])
    left_voltage = float(values[2]) if len(values) >= 3 else left * 3.3
    right_voltage = float(values[3]) if len(values) >= 4 else right * 3.3
    raw_left = int(values[4]) if len(values) >= 5 else None
    raw_right = int(values[5]) if len(values) >= 6 else None
    return FootLoadReading(left, right, left_voltage, right_voltage), raw_left, raw_right


def format_reading(reading: FootLoadReading, raw_left: int | None, raw_right: int | None) -> str:
    raw_text = ""
    if raw_left is not None and raw_right is not None:
        raw_text = f" raw L={raw_left:4d} R={raw_right:4d}"
    return (
        f"L={reading.left:.3f} ({reading.left_voltage:.3f}V) "
        f"R={reading.right:.3f} ({reading.right_voltage:.3f}V) "
        f"ratio L={reading.left_ratio:.2f} R={reading.right_ratio:.2f} "
        f"total={reading.total:.3f}{raw_text}"
    )


def main() -> None:
    cfg = Config()
    parser = argparse.ArgumentParser(description="Read FSR packets from the ESP32 USB serial stream.")
    parser.add_argument("--port", default=cfg.sensor_port)
    parser.add_argument("--baudrate", type=int, default=cfg.sensor_baudrate)
    parser.add_argument("--warn-after", type=float, default=3.0)
    args = parser.parse_args()

    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("FSR test requires: pip install pyserial") from exc

    print(f"FSR serial test: port={args.port}, baudrate={args.baudrate}")
    print("Expected firmware packet: F,ms,left,right,left_voltage,right_voltage,left_raw,right_raw")
    print("Press Ctrl+C to stop.\n")

    with serial.Serial(args.port, args.baudrate, timeout=1.0) as ser:
        last_packet_t = time.monotonic()
        line_count = 0
        try:
            while True:
                raw = ser.readline().decode("utf-8", errors="replace")
                line_count += 1 if raw else 0
                try:
                    parsed = parse_fsr_packet(raw)
                except ValueError:
                    parsed = None

                if parsed is None:
                    if time.monotonic() - last_packet_t >= args.warn_after:
                        print(f"waiting for FSR packet... serial lines seen={line_count}")
                        last_packet_t = time.monotonic()
                    continue

                last_packet_t = time.monotonic()
                reading, raw_left, raw_right = parsed
                print(format_reading(reading, raw_left, raw_right))
        except KeyboardInterrupt:
            print("\nFSR test stopped.")


if __name__ == "__main__":
    main()
