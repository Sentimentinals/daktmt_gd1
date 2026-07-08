# Applied References for the Humanoid Robot Project

This document maps external projects, documentation, and research papers to concrete design decisions in this 17-DOF humanoid robot.

## 1. Reference Mapping

| Source | Type | Relevant idea | How it is applied here |
| :--- | :--- | :--- | :--- |
| Raspberry Pi 5 documentation | Official documentation | Pi 5 requires a stable high-current 5.1V supply and has limited GPIO/sensor current budget | Raspberry Pi uses its own 5.1V/5A power domain, separated from the servo rail |
| Adafruit PCA9685 servo driver guide | Hardware documentation | Servo power must be separate from logic power; high-torque servos create current spikes and brownout risk | Servo controller and 35 kg-cm servos use a separate 6V high-current buck with bulk capacitance |
| Adafruit BNO055 guide | Sensor documentation | BNO055 provides onboard orientation fusion and quaternion/Euler output | BNO055 roll/pitch is used for bounded PID balance correction |
| Adafruit ADS1115 guide | Sensor documentation | Raspberry Pi needs an external ADC for analog sensors | ADS1115 reads FSR foot-load dividers over I2C |
| LIKO biped odometry paper | Research paper | Biped state estimation benefits from inertial and contact-state information | The project uses FSR support-load validation before allowing swing |
| `pib.rocks` | Open-source humanoid project | Raspberry Pi can act as the high-level Linux controller in a modular humanoid | Raspberry Pi 5 performs gait generation, sensor processing, and user input handling |
| Poppy Project | Open-source humanoid platform | High-level robot behavior can be separated from actuator electronics | The walking engine stays independent from the serial servo backend |
| Salvius | Open-source humanoid architecture | Raspberry Pi can coordinate distributed low-level microcontroller/sensor nodes | Future extension can move IMU/FSR filtering to STM32/ESP32 without changing the gait planner |

## 2. Resulting System Architecture

```text
User input
  -> Raspberry Pi 5
      -> walking engine
      -> BNO055 roll/pitch feedback
      -> ADS1115/FSR support-load validation
      -> bounded balance PID
  -> USB/UART serial
  -> 32-channel servo controller
  -> 17 PWM servos
```

Power distribution:

```text
LiPo 3S
  -> relay / emergency cut
  -> 6V high-current buck -> servo controller + servos
  -> 5.1V 5A buck or USB-C PD -> Raspberry Pi 5
  -> Pi 3.3V -> BNO055 + ADS1115 + FSR voltage dividers
```

## 3. Thesis Wording

Recommended wording:

```text
The proposed robot adopts an application-oriented modular control architecture. Raspberry Pi 5 is used as the high-level controller, while the servo controller handles PWM signal generation for the 17 actuators. To reduce power interference, the actuator rail, computing rail, and sensor rail are separated and connected through a common ground reference. Balance feedback is obtained from a BNO055 IMU, while FSR sensors connected through an ADS1115 ADC provide foot-load information for support-state validation.
```

Algorithm wording:

```text
The walking controller combines a simplified LIPM/ZMP-inspired gait reference, inverse-kinematics-based foot trajectory generation, bounded IMU postural correction, and FSR-based support-foot validation. This architecture prioritizes repeatable physical operation on a low-cost PWM-servo humanoid platform rather than full whole-body dynamic optimization.
```

## 4. Practical Design Decisions

- Use BNO055 fusion first because it is more reliable for an application-focused prototype than adding a custom EKF before the robot can walk consistently.
- Use FSR support validation because the key walking failure mode is lifting a foot before the support side carries enough body weight.
- Use a separate 6V actuator supply because 35 kg-cm servos generate current spikes that can reset or corrupt the Raspberry Pi if power is shared incorrectly.
- Keep all sensors on the 3.3V Pi domain to avoid level-shifting complexity.
- Keep the serial servo controller as the only actuator backend to remain compatible with the existing 32-channel PWM architecture.

## 5. Future Extension

If BNO055 fusion becomes unreliable during fast motion, an STM32 or ESP32 sensor co-processor can run EKF-based raw IMU filtering and send filtered roll/pitch to the Raspberry Pi. This should be treated as a sensor-estimation upgrade, not a replacement for the walking engine.

