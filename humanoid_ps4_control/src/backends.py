from __future__ import annotations

import csv
import time
from pathlib import Path
from typing import Dict, Optional


Pose = Dict[int, int]


def _build_rt_command(pose: Pose, duration_ms: int = 1000) -> str:
    """Build one RTrobot command: #1P1500#2P1500...T40D0."""
    parts = [f"#{sid}P{pulse}" for sid, pulse in sorted(pose.items())]
    return "".join(parts) + f"T{duration_ms}D0"


class MockBackend:
    """Print command strings without touching hardware."""

    def __init__(self, verbose: bool = True) -> None:
        self.verbose = verbose
        self._frame_count = 0

    def __enter__(self) -> "MockBackend":
        self.open()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    def open(self) -> None:
        print("[MockBackend] Opened (no hardware)")

    def send(self, pose: Pose, duration_ms: int = 1000, force: bool = False) -> None:
        self._frame_count += 1
        cmd = _build_rt_command(pose, duration_ms)
        if self.verbose:
            preview = cmd[:77] + "..." if len(cmd) > 80 else cmd
            print(f"[MOCK #{self._frame_count:04d}] {preview}")

    def close(self) -> None:
        print(f"[MockBackend] Closed after {self._frame_count} frames sent")


class SerialRTBackend:
    """
    Send RTrobot serial commands via pyserial.

    The 32-channel servo board has a small UART buffer, so each full pose is
    split into small command chunks.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 1.0,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial = None
        self._prev_pose: Pose = {}
        self._frame_count = 0

    def __enter__(self) -> "SerialRTBackend":
        self.open()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    def open(self) -> None:
        try:
            import serial  # type: ignore
            from serial import SerialException  # type: ignore
        except ImportError as exc:
            raise ImportError("pyserial is required. Install with: pip install pyserial") from exc

        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
            )
        except SerialException as exc:
            raise RuntimeError(
                f"Cannot open servo controller port {self.port}. "
                "On Raspberry Pi, keep ESP32 sensor on /dev/ttyUSB0 and set the servo "
                "controller port to the other USB device, usually /dev/ttyUSB1."
            ) from exc
        print(f"[SerialRTBackend] Opened {self.port} @ {self.baudrate} baud")

    def send(self, pose: Pose, duration_ms: int = 1000, force: bool = False) -> None:
        if self._serial is None or not self._serial.is_open:
            raise RuntimeError("Serial port is not open. Call open() first.")

        self._frame_count += 1
        is_moving = pose != self._prev_pose

        # When standing still, refresh occasionally instead of streaming a
        # duplicate 25 Hz pose forever. This reduces load on the servo board.
        if not force and not is_moving and self._frame_count % 10 != 0:
            return

        self._prev_pose = dict(pose)

        # Send all active servos in every moving frame. This prevents hidden
        # action-group playback on the board from taking control of untouched
        # channels, while chunking keeps each UART write below the FIFO limit.
        items = sorted(pose.items())
        batch_size = 5
        for i in range(0, len(items), batch_size):
            batch = dict(items[i : i + batch_size])
            cmd = _build_rt_command(batch, duration_ms) + "\r\n"
            self._serial.write(cmd.encode("ascii"))
            self._serial.flush()
            time.sleep(0.002)

    def close(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()
            print(f"[SerialRTBackend] Closed {self.port}")


class CsvLogBackend:
    """Append timestamped pose rows to a CSV file."""

    def __init__(self, csv_path: str | Path) -> None:
        self.csv_path = Path(csv_path)
        self._file = None
        self._writer: Optional[csv.DictWriter] = None
        self._fieldnames: Optional[list[str]] = None
        self._start_time = time.time()

    def __enter__(self) -> "CsvLogBackend":
        self.open()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    def open(self) -> None:
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self._start_time = time.time()
        print(f"[CsvLogBackend] Writing to {self.csv_path}")

    def send(self, pose: Pose, duration_ms: int = 1000, force: bool = False) -> None:
        if self._file is None:
            raise RuntimeError("CsvLogBackend is not open. Call open() first.")

        timestamp = round(time.time() - self._start_time, 4)
        if self._writer is None:
            servo_cols = [f"servo_{sid}" for sid in sorted(pose.keys())]
            self._fieldnames = ["timestamp_s", "duration_ms"] + servo_cols
            self._writer = csv.DictWriter(self._file, fieldnames=self._fieldnames)
            self._writer.writeheader()

        row: Dict[str, float | int] = {"timestamp_s": timestamp, "duration_ms": duration_ms}
        for sid, pulse in sorted(pose.items()):
            row[f"servo_{sid}"] = pulse

        self._writer.writerow(row)

    def close(self) -> None:
        if self._file:
            self._file.flush()
            self._file.close()
            print(f"[CsvLogBackend] Closed {self.csv_path}")


def make_backend(
    mode: str,
    port: str = "/dev/ttyUSB0",
    baudrate: int = 115200,
    csv_path: str | Path = "out/log.csv",
    verbose: bool = True,
):
    if mode == "mock":
        return MockBackend(verbose=verbose)
    if mode == "serial":
        return SerialRTBackend(port=port, baudrate=baudrate)
    if mode == "csv":
        return CsvLogBackend(csv_path=csv_path)
    raise ValueError(f"Unknown backend mode: {mode!r}. Use mock|serial|csv")
