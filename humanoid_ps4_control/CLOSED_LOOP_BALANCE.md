# Closed-loop IMU Balance

This project now supports optional BNO055 roll/pitch feedback for the real-time ZMP walking engine.

## Control Path

```text
PS4 input
  -> DynamicWalkingEngine
  -> ZMP preview + leg IK
  -> BNO055 roll/pitch PID correction
  -> ankle/hip PWM offsets
  -> serial servo controller
```

The closed-loop layer is intentionally small and bounded. It does not replace the ZMP planner; it only adds limited ankle/hip corrections to reject real hardware tilt errors.

## Files

- `src/imu_bno055.py`: BNO055 reader using `adafruit-circuitpython-bno055`
- `src/balance.py`: PID controller and ankle/hip correction mixer
- `src/main.py`: `--imu-balance` CLI wiring
- `src/walking_engine.py`: exposes `engine.support_leg` for support-weighted correction

## Run

Mock or serial walking still runs without IMU by default:

```bash
python -m src.main --ps4 --backend mock
```

Enable BNO055 balance on Raspberry Pi:

```bash
python -m src.main --ps4 --backend serial --port /dev/ttyUSB0 --imu-balance
```

If the IMU is mounted in the opposite direction, flip signs one axis at a time:

```bash
python -m src.main --ps4 --backend serial --port /dev/ttyUSB0 --imu-balance --imu-pitch-sign -1
python -m src.main --ps4 --backend serial --port /dev/ttyUSB0 --imu-balance --imu-roll-sign -1
```

## Initial Tuning Procedure

1. Suspend or hand-support the robot.
2. Run standing / stop mode first, with `--balance-limit-deg 3`.
3. Tilt the robot forward by hand and verify ankle/hip motion pushes back, not further into the fall.
4. Fix axis signs before increasing gains or walking.
5. Test double support, then slow stepping.

Recommended first run:

```bash
python -m src.main --ps4 --backend serial --port /dev/ttyUSB0 --imu-balance --balance-limit-deg 3
```

Raise to `5-6` degrees only after the correction direction is verified.

## Current Assumptions

- IMU convention after sign mapping:
  - positive roll = robot leans left
  - positive pitch = robot leans forward
- Servo IDs follow `PROJECT_ARCHITECTURE.md`.
- Correction is applied after IK as PWM offsets. This is practical for MG996R hardware and avoids destabilizing the existing gait planner.
