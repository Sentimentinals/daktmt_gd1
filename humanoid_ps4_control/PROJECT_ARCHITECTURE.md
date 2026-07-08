# 17-DOF Humanoid Robot - Real-Time ZMP Walking Engine
> **Note for AI Agents/Models:** Read this document carefully to understand the system architecture, hardware constraints, and coordinate conventions before modifying any code.

## 1. Project Overview
This project controls a 17-DOF bipedal humanoid robot. It transitions the robot from playing static keyframed XML animations to a **Real-Time Dynamic Walking Engine** based on Kajita's ZMP (Zero Moment Point) Preview Control. The robot is driven by a PS4 D-Pad (via Pygame) and computes Inverse Kinematics (IK) at runtime.

### Hardware Stack
*   **Brain:** Raspberry Pi 5 / Windows PC
*   **Controller:** 32-Channel Serial Servo Controller (RTrobot protocol or similar clone)
*   **Actuators:** 17x standard-size 180-degree PWM servos. The current hardware target is the upgraded 35 kg-cm class servo set.
*   **Balance Sensors:** BNO055 IMU on I2C, ADS1115 ADC on I2C, and one FSR load sensor per foot.
*   **Communication:** USB-to-UART (Serial) at `115200 bps`

### Power Domains
The robot must keep actuator power, computing power, and sensor power separated. This follows the same practical rule used by Raspberry Pi robotics platforms and servo-driver references: high-current actuators must not draw through the Raspberry Pi power rail.

```text
LiPo 3S
  -> relay / emergency cut
  -> 6.0V high-current buck -> 32-channel servo controller Vservo + 17 servos
  -> 5.1V 5A buck or USB-C PD supply -> Raspberry Pi 5
  -> Pi 3.3V rail -> BNO055 + ADS1115 + FSR voltage dividers
```

Grounding rule:

```text
servo GND, Pi GND, and sensor GND meet at one star-ground point.
Servo current must not return through Raspberry Pi sensor wiring.
```

---

## 2. Hardware Mapping (Servo IDs)
The robot uses exactly 17 channels on the 32-channel board. **Do not alter these mappings without explicit user instruction.**

| Body Part | Right Side (ID) | Left Side (ID) | Center (ID) |
| :--- | :--- | :--- | :--- |
| **Foot (Ankle Roll)** | 1 | 24 | |
| **Ankle Pitch** | 2 | 23 | |
| **Knee** | 3 | 22 | |
| **Thigh (Hip Pitch)** | 4 | 21 | |
| **Hip (Abduct/Roll)** | 5 | 20 | |
| **Elbow** | 6 | 19 | |
| **Upper Arm** | 7 | 18 | |
| **Shoulder** | 8 | 17 | |
| **Head (Neck)** | | | 16 (or 17) |

---

## 3. Core Architecture & Modules

### `src/main.py`
*   Entry point. Uses Pygame to read PS4 D-Pad inputs (or Keyboard fallback: Up/Down/C/Q).
*   Translates inputs into a desired velocity (`vy`).
*   The Pygame loop `poll_rate_hz` **must match** `1000 / update_ms`. The current 35 kg-cm servo configuration uses `update_ms = 30` by default.

### `src/walking_engine.py` (DynamicWalkingEngine)
*   The orchestrator. Maintains the `STANDING` pose constants.
*   **Treadmill Logic:** Keeps the Center of Mass (CoM) fixed at `(0, 0, 147.4)` while the footprints move backward relative to the CoM.
*   Calls ZMP controller to calculate lateral swaying (`com_y`).
*   Generates smooth swing-foot trajectories. The current 35 kg-cm servo configuration uses `step_height = 34mm` by default.
*   **`DIR` Map:** Defines the physical rotation polarity for symmetric joints.
*   Uses FSR support-leg validation when sensor feedback is enabled. A swing step is delayed if the planned support foot has not reached the configured load ratio.

### `src/zmp_controller.py`
*   Implements Kajita's 3D Linear Inverted Pendulum Model (LIPM) and LQR Preview Control.
*   Takes upcoming footprint positions and generates smooth CoM trajectories.
*   **Math Fix:** The discrete Riccati equation `Aa` matrix and error integral accumulation sign were corrected to prevent numerical divergence.

### `src/leg_ik.py` & `src/leg_fk.py`
*   **IK:** Computes joint angles based on desired foot cartesian coordinates relative to the hip. Note: Knee angle convention is `180 - gamma` (0 = fully straight).
*   **FK:** Uses Denavit-Hartenberg (D-H) parameters to map angles back to Cartesian space (useful for thesis plotting/validation).

### `src/backends.py`
*   Handles Serial communication formatted as `#1P1500...T40D0\r\n`.
*   Implements a `_prev_pose` cache to **only send deltas** (servos that moved) to conserve USB bandwidth.

### `src/sensors.py`
*   Reads the BNO055 IMU and ADS1115 foot-load channels.
*   Normalizes left/right FSR readings and exposes support-load ratios for the walking engine.
*   Keeps the sensor layer separate from gait generation so the same walking engine can run in mock mode, open-loop mode, or closed-loop mode.

---

## 4. Critical Hardware Constraints & Gotchas (SOLVED)

Future models must be aware of these hardware limitations before modifying the update rate or serial backend:

### A. UART RX Buffer Overflow (The "Left Side Freeze" Bug)
*   **Issue:** The 32-channel controller has a tiny hardware UART FIFO buffer (likely 64 or 128 bytes). If the Python script sends all 17 servos at 50Hz (20ms), the string length (`~113 bytes`) exceeds the microcontroller's parsing speed. The buffer overflows, truncating the tail end of the string. Since the string is sorted by ID, **servos 25-32 (the entire left leg) were randomly freezing**.
*   **Solution:** The legacy MG996R tuning used **25Hz (`update_ms = 40`)**. The 35 kg-cm servo tuning now uses **about 33Hz (`update_ms = 30`)**. If left-leg freezing or serial truncation returns, increase `update_ms` back to `40` before changing gait logic.

### B. Controller Boot Delay (The "Misaligned Head" Bug)
*   **Issue:** Opening the COM port on Windows toggles DTR, resetting the microcontroller (takes ~1-2s to boot). The first frame sent by Python contains the `Head (16)` position. Because the MCU was booting, it missed this frame. Due to the delta-cache logic in `backends.py`, static servos like the Head were never sent again, causing misalignment.
*   **Solution:** `backends.py` now implements a **Full Sync**. Every 50 frames, it clears the cache and forces a transmission of all servos. It also forces sending static IDs (16, 17) every frame to overpower any internal Action Groups running on the controller.

### C. Internal Action Group Conflicts
*   **Issue:** Some controller clones run a default Action Group 0 on boot, which continuously spams default PWMs (e.g., 1500) internally. This fights with Python's serial commands, causing extreme jitter or unresponsive servos.
*   **Solution:** Do **not** send the `PS\r\n` (Play Stop) command, as it completely froze this specific user's clone firmware. Instead, we overpower the internal group by sending our trajectory points continuously at 25Hz.

---

## 5. Running the System
```bash
python -m src.main
```
The default `Config` uses serial mode, PS4/keyboard input, IMU balance, and FSR support validation. If no PS4 controller is detected, Pygame opens a keyboard fallback window. Focus the window, then use `W/S/A/D` or arrow keys.

---

## 6. Reference Projects Applied

The project uses external humanoid and Raspberry Pi robotics projects as architectural references, not as direct source-code dependencies.

| Reference | What it contributes | Applied design decision |
| :--- | :--- | :--- |
| `pib.rocks` open-source humanoid | Raspberry Pi 5 as high-level Linux/ROS-class computer, modular sensors/electronics | Keep Raspberry Pi as the high-level gait, sensor, and application controller |
| Poppy Project | Raspberry Pi-based humanoid software stack with high-level Python/ROS/web control | Separate high-level control from actuator electronics |
| Salvius | Raspberry Pi plus distributed microcontroller nodes | Keep the option to add STM32/ESP32 as a sensor co-processor if BNO055/FSR timing becomes unreliable |
| Adafruit PCA9685 servo-driver guide | Servo power must be separate from logic power; large servos can draw high peak current | Keep 6V servo power separate from Pi 5V; use common ground and bulk capacitors |
| Raspberry Pi 5 official docs | Pi 5 needs a strong 5.1V supply; GPIO/sensor current is limited | Use a dedicated 5.1V/5A Pi supply and do not power actuators from Pi |
| BNO055 documentation | IMU provides fused orientation, quaternion/Euler, gyro, acceleration, gravity vector | Use BNO055 roll/pitch for bounded closed-loop balance rather than implementing custom EKF first |
| ADS1115 documentation | Raspberry Pi needs an external ADC for analog FSR readings | Read foot pressure through ADS1115 over I2C |
| LIKO biped state-estimation paper | Foot contact and inertial data are important for biped state estimation | Use FSR readings to validate support-foot loading before swing |

These references support the thesis-level architecture claim:

```text
The robot uses a sensor-assisted closed-loop gait architecture:
model-based walking reference + IMU postural correction + FSR support validation.
```
