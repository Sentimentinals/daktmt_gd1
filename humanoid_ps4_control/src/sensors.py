from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .imu_bno055 import BNO055Reader, IMUReading


@dataclass(frozen=True)
class FootLoadReading:
    left: float
    right: float
    left_voltage: float = 0.0
    right_voltage: float = 0.0

    @property
    def total(self) -> float:
        return max(0.0, self.left + self.right)

    @property
    def left_ratio(self) -> float:
        total = self.total
        return 0.5 if total <= 1e-6 else self.left / total

    @property
    def right_ratio(self) -> float:
        total = self.total
        return 0.5 if total <= 1e-6 else self.right / total


@dataclass(frozen=True)
class SensorSnapshot:
    imu: Optional[IMUReading]
    foot_load: Optional[FootLoadReading]


class LowPass:
    def __init__(self, alpha: float) -> None:
        self.alpha = max(0.01, min(1.0, alpha))
        self.value: Optional[float] = None

    def update(self, sample: float) -> float:
        if self.value is None:
            self.value = sample
        else:
            self.value = self.value + self.alpha * (sample - self.value)
        return self.value


class ADS1115FootLoadReader:
    def __init__(
        self,
        address: int = 0x48,
        left_channel: int = 0,
        right_channel: int = 1,
        invert: bool = False,
        filter_alpha: float = 0.18,
    ) -> None:
        self.address = address
        self.left_channel = left_channel
        self.right_channel = right_channel
        self.invert = invert
        self.left_filter = LowPass(filter_alpha)
        self.right_filter = LowPass(filter_alpha)
        self._channels = None

    def open(self) -> None:
        try:
            import board  # type: ignore
            import busio  # type: ignore
            import adafruit_ads1x15.ads1115 as ADS  # type: ignore
            from adafruit_ads1x15.analog_in import AnalogIn  # type: ignore
        except ImportError as exc:
            raise ImportError(
                "FSR support requires: pip install adafruit-circuitpython-ads1x15"
            ) from exc

        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1115(i2c, address=self.address)
        pins = [ADS.P0, ADS.P1, ADS.P2, ADS.P3]
        self._channels = (
            AnalogIn(ads, pins[self.left_channel]),
            AnalogIn(ads, pins[self.right_channel]),
        )

    def read(self) -> FootLoadReading:
        if self._channels is None:
            raise RuntimeError("ADS1115FootLoadReader.open() must be called before read().")

        left_voltage = self._channels[0].voltage
        right_voltage = self._channels[1].voltage
        left = self._normalize(left_voltage)
        right = self._normalize(right_voltage)
        return FootLoadReading(
            left=self.left_filter.update(left),
            right=self.right_filter.update(right),
            left_voltage=left_voltage,
            right_voltage=right_voltage,
        )

    def _normalize(self, voltage: float) -> float:
        value = max(0.0, min(1.0, float(voltage) / 3.3))
        return 1.0 - value if self.invert else value


class RobotSensorHub:
    def __init__(
        self,
        use_imu: bool = True,
        use_fsr: bool = True,
        imu_roll_sign: float = 1.0,
        imu_pitch_sign: float = 1.0,
        imu_yaw_sign: float = 1.0,
        fsr_ads1115_address: int = 0x48,
        fsr_left_channel: int = 0,
        fsr_right_channel: int = 1,
        fsr_invert: bool = False,
        fsr_filter_alpha: float = 0.18,
    ) -> None:
        self.imu = (
            BNO055Reader(
                roll_sign=imu_roll_sign,
                pitch_sign=imu_pitch_sign,
                yaw_sign=imu_yaw_sign,
            )
            if use_imu
            else None
        )
        self.fsr = (
            ADS1115FootLoadReader(
                address=fsr_ads1115_address,
                left_channel=fsr_left_channel,
                right_channel=fsr_right_channel,
                invert=fsr_invert,
                filter_alpha=fsr_filter_alpha,
            )
            if use_fsr
            else None
        )

    def open(self) -> None:
        if self.imu is not None:
            self.imu.open()
        if self.fsr is not None:
            self.fsr.open()

    def read(self) -> SensorSnapshot:
        imu_reading = self.imu.read() if self.imu is not None else None
        foot_load = self.fsr.read() if self.fsr is not None else None
        return SensorSnapshot(imu=imu_reading, foot_load=foot_load)
