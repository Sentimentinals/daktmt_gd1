# 17-DOF Humanoid Robot - Real-Time ZMP Walking Engine
> **Note for AI Agents/Models:** Read this document carefully to understand the system architecture, hardware constraints, and coordinate conventions before modifying any code.

## 1. Project Overview
This project controls a 17-DOF bipedal humanoid robot. It transitions the robot from playing static keyframed XML animations to a **Real-Time Dynamic Walking Engine** based on Kajita's ZMP (Zero Moment Point) Preview Control. The robot is driven by a PS4 D-Pad (via Pygame) and computes Inverse Kinematics (IK) at runtime.

### Hardware Stack
*   **Brain:** Raspberry Pi 5 / Windows PC
*   **Controller:** 32-Channel Serial Servo Controller (RTrobot protocol or similar clone)
*   **Actuators:** 17x MG996R (or similar 180-degree RC servos)
*   **Communication:** USB-to-UART (Serial) at `115200 bps`

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
*   The Pygame loop `poll_rate_hz` **must match** `1000 / update_ms` (currently 25Hz / 40ms) to prevent USB buffer overflow.

### `src/walking_engine.py` (DynamicWalkingEngine)
*   The orchestrator. Maintains the `STANDING` pose constants.
*   **Treadmill Logic:** Keeps the Center of Mass (CoM) fixed at `(0, 0, 147.4)` while the footprints move backward relative to the CoM.
*   Calls ZMP controller to calculate lateral swaying (`com_y`).
*   Generates cycloid trajectories for the swing foot (`step_height = 25mm`).
*   **`DIR` Map:** Defines the physical rotation polarity for symmetric joints.

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

---

## 4. Critical Hardware Constraints & Gotchas (SOLVED)

Future models must be aware of these hardware limitations before modifying the update rate or serial backend:

### A. UART RX Buffer Overflow (The "Left Side Freeze" Bug)
*   **Issue:** The 32-channel controller has a tiny hardware UART FIFO buffer (likely 64 or 128 bytes). If the Python script sends all 17 servos at 50Hz (20ms), the string length (`~113 bytes`) exceeds the microcontroller's parsing speed. The buffer overflows, truncating the tail end of the string. Since the string is sorted by ID, **servos 25-32 (the entire left leg) were randomly freezing**.
*   **Solution:** The system now runs at **25Hz (`update_ms = 40`)**. This halves the data rate, giving the MCU 30ms of idle time to parse the string. The robot still walks perfectly smoothly because the controller interpolates `T40`.

### B. Controller Boot Delay (The "Misaligned Head" Bug)
*   **Issue:** Opening the COM port on Windows toggles DTR, resetting the microcontroller (takes ~1-2s to boot). The first frame sent by Python contains the `Head (16)` position. Because the MCU was booting, it missed this frame. Due to the delta-cache logic in `backends.py`, static servos like the Head were never sent again, causing misalignment.
*   **Solution:** `backends.py` now implements a **Full Sync**. Every 50 frames, it clears the cache and forces a transmission of all servos. It also forces sending static IDs (16, 17) every frame to overpower any internal Action Groups running on the controller.

### C. Internal Action Group Conflicts
*   **Issue:** Some controller clones run a default Action Group 0 on boot, which continuously spams default PWMs (e.g., 1500) internally. This fights with Python's serial commands, causing extreme jitter or unresponsive servos.
*   **Solution:** Do **not** send the `PS\r\n` (Play Stop) command, as it completely froze this specific user's clone firmware. Instead, we overpower the internal group by sending our trajectory points continuously at 25Hz.

---

## 5. Running the System
```bash
python -m src.main --backend serial --port COM24 --ps4
```
*(If no PS4 controller is detected, Pygame will open a 400x200 window. **Click this window to focus**, then use Keyboard `Up Arrow` to walk, `C` to stop).*
