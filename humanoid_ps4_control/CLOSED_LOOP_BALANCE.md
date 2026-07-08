# Closed-loop IMU/FSR Balance

This project supports BNO055 roll/pitch feedback and FSR foot-load validation for the real-time ZMP walking engine.

## Control Path

```text
PS4 input
  -> DynamicWalkingEngine
  -> ZMP preview + leg IK + support/swing phase generation
  -> FSR support-foot load validation
  -> BNO055 roll/pitch PID postural correction
  -> ankle/hip PWM offsets
  -> serial servo controller
```

The closed-loop layer is intentionally bounded. It does not replace the gait planner; it only:

- delays swing if the intended support foot is not carrying enough load,
- adds limited ankle/hip corrections to reject real hardware tilt errors.

## Files

- `src/imu_bno055.py`: BNO055 reader using `adafruit-circuitpython-bno055`
- `src/sensors.py`: BNO055 + ADS1115/FSR sensor hub
- `src/balance.py`: PID controller and ankle/hip correction mixer
- `src/main.py`: sensor hub wiring and real-time feedback loop
- `src/walking_engine.py`: exposes `engine.support_leg` and validates support-foot load before swing

## Run

The current application-oriented configuration enables sensor feedback by default in `src/config.py`.

Run on the Raspberry Pi with BNO055 and ADS1115 connected:

```bash
python -m src.main
```

To test without FSR while keeping IMU balance, set:

```python
sensor_feedback = True
sensor_use_imu = True
sensor_use_fsr = False
```

To test without any sensor feedback, set:

```python
imu_balance = False
sensor_feedback = False
```

## Initial Tuning Procedure

1. Suspend or hand-support the robot.
2. Confirm I2C devices with `i2cdetect -y 1`:
   - BNO055: usually `0x28` or `0x29`
   - ADS1115: default `0x48`
3. Enable `sensor_debug` and verify:
   - pressing left foot increases left FSR ratio,
   - pressing right foot increases right FSR ratio.
4. Run standing mode first and verify BNO055 roll/pitch signs.
5. Tilt the robot forward by hand and verify ankle/hip motion pushes back, not further into the fall.
6. Fix axis signs before increasing gains or walking.
7. Test double support, single-support, then slow stepping.

Recommended first sensor run:

```python
sensor_debug = True
balance_limit_deg = 3.0
```

Raise to `5` degrees only after the correction direction is verified.

## Current Assumptions

- IMU convention after sign mapping:
  - positive roll = robot leans left
  - positive pitch = robot leans forward
- FSR convention:
  - `left_ratio = left / (left + right)`
  - `right_ratio = right / (left + right)`
  - swing is allowed only when the planned support side reaches `fsr_support_ratio`
- Servo IDs follow `PROJECT_ARCHITECTURE.md`.
- Correction is applied after IK as PWM offsets. This is practical for PWM-servo hardware and avoids destabilizing the existing gait planner.

## Reference Basis

- BNO055 is used because it provides onboard sensor fusion and outputs roll/pitch without requiring a custom EKF in the first deployment.
- ADS1115 is used because Raspberry Pi has no analog input, while FSR sensors are analog resistive sensors.
- Foot-load validation follows the biped state-estimation principle that contact state should gate support/swing transitions.
