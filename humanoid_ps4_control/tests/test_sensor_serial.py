from __future__ import annotations

import unittest

from src.sensors import RobotSensorHub


class FakeSerial:
    def __init__(self, lines: list[bytes]) -> None:
        self.lines = iter(lines)

    def readline(self) -> bytes:
        return next(self.lines, b"")


class SensorSerialTests(unittest.TestCase):
    def test_probe_accepts_fsr_packet_even_when_i2c_sensors_are_missing(self) -> None:
        serial_port = FakeSerial([b"# BNO055 setup failed.\n", b"F,20,0.1,0.33,410,0.2,0.66,820\n"])
        self.assertTrue(RobotSensorHub._probe_sensor_stream(serial_port, timeout_s=0.01))

    def test_probe_rejects_boot_log_and_unrelated_serial_data(self) -> None:
        serial_port = FakeSerial([b"rst:0x1 (POWERON_RESET)\n", b"servo ready\n"])
        self.assertFalse(RobotSensorHub._probe_sensor_stream(serial_port, timeout_s=0.01))


if __name__ == "__main__":
    unittest.main()
