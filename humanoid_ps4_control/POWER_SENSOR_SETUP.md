# Power And Sensor Setup

## Power Domains

Use three separated power domains with one shared ground point.

```text
LiPo 3S
  -> relay / emergency cut
  -> 6.0V high-current buck -> 32-channel servo controller + servos
  -> 5.1V 5A buck          -> Raspberry Pi
  -> Pi 3.3V               -> BNO055 + ADS1115 + FSR dividers
```

Do not power the Raspberry Pi from the 6V servo rail. Do not power servos from the Pi.

## I2C Wiring

```text
Pi GPIO2 / Pin 3 -> SDA -> BNO055 SDA + ADS1115 SDA
Pi GPIO3 / Pin 5 -> SCL -> BNO055 SCL + ADS1115 SCL
Pi 3.3V          -> BNO055 VIN/VCC + ADS1115 VDD
Pi GND           -> BNO055 GND + ADS1115 GND
ADS1115 ADDR     -> GND  # address 0x48
```

Enable I2C on the Pi:

```bash
sudo raspi-config
i2cdetect -y 1
```

Expected addresses:

```text
0x28 or 0x29 -> BNO055
0x48         -> ADS1115
```

## FSR Wiring

Each FSR uses one voltage divider.

```text
3.3V -> FSR -> ADS1115 A0/A1 -> 10k resistor -> GND
```

Mapping:

```text
Left foot  FSR -> ADS1115 A0
Right foot FSR -> ADS1115 A1
```

In `src/config.py`:

```python
sensor_feedback = True
sensor_use_imu = True
sensor_use_fsr = True
fsr_ads1115_address = 0x48
fsr_left_channel = 0
fsr_right_channel = 1
```

## Sensor Test

Run from project root:

```bash
python tools/sensor_check.py
```

Expected behavior:

```text
Press left FSR  -> L value and L ratio increase
Press right FSR -> R value and R ratio increase
Tilt robot      -> roll/pitch changes
```

If FSR values move in the wrong direction, set:

```python
fsr_invert = True
```
