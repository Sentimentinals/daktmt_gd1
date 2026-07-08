# Algorithm Report Notes

This document states exactly what the current code implements. It is written so the thesis/report does not overclaim the algorithmic level.

## Walking Model

The walking engine uses a Linear Inverted Pendulum Model (LIPM) in the horizontal plane with constant CoM height `zc`.

For one horizontal axis:

```text
p = c - (zc / g) * c_ddot
```

where `p` is ZMP, `c` is CoM position, `c_ddot` is CoM acceleration, and `g` is gravity.

The discrete state is:

```text
x[k] = [c[k], c_dot[k], c_ddot[k]]^T
u[k] = c_jerk[k]

x[k+1] = A x[k] + B u[k]
p[k]   = C x[k]
```

with:

```text
A = [[1, dt, dt^2/2],
     [0,  1, dt],
     [0,  0, 1]]

B = [[dt^3/6],
     [dt^2/2],
     [dt]]

C = [1, 0, -zc/g]
```

`src/zmp_controller.py` implements the LQ preview controller with integrated ZMP tracking error and a finite preview horizon. The preview gain recursion follows:

```text
Gp(j) = inv(R + B_aug^T P B_aug) B_aug^T (Ac^T)^(j-1) P I
```

where `P` is obtained by Riccati iteration and `Ac = A_aug - B_aug K`.

## Current Scope

`src/walking_engine.py` currently applies ZMP preview control to the lateral CoM coordinate. The sagittal step motion is generated kinematically by swing/stance foot trajectories. Therefore, the correct report wording is:

```text
The system implements lateral LIPM/ZMP preview stabilization combined with inverse-kinematics-based foot trajectory generation.
```

Do not claim full 3D whole-body dynamics or full x/y ZMP optimal control unless sagittal CoM preview is added later.

## Foot Trajectory

During single support, the swing foot follows:

```text
s = 6a^5 - 15a^4 + 10a^3
z = step_height * sin(pi * s)
```

where `a` is normalized phase from `0` to `1`. This gives zero vertical lift at touchdown/liftoff and smooth mid-swing clearance.

## Leg IK

`src/leg_ik.py` separates:

- frontal-plane hip abduction from `(dy, dz)`
- sagittal hip/knee/ankle pitch from `(dx, leg_len_frontal)`

Main equations:

```text
hip_abduct = atan2(dy, -dz)
L = sqrt(dx^2 + dy^2 + dz^2)
knee = 180deg - acos((L1^2 + L2^2 - L^2) / (2 L1 L2))
hip_pitch = atan2(dx, sqrt(dy^2 + dz^2)) + acos((L1^2 + L^2 - L2^2) / (2 L1 L))
ankle_pitch = knee - hip_pitch
```

`src/leg_fk.py` was aligned with this convention for the ankle/foot-point position
and is validated by `tools/validate_algorithms.py`.

Important scope boundary: the current IK/FK validates endpoint position, not a
full 6D foot pose. `ankle_pitch` is a calibrated pitch-compensation signal that
matches the existing servo direction table and standing baseline. If the thesis
needs strict foot-orientation control, add an explicit foot orientation state and
derive the ankle equation from the mounted servo frame.

## Sensor-Assisted Closed Loop

`src/balance.py` is a bounded PID correction layer applied after IK. It does not replace the gait planner. It adjusts ankle/hip PWM around the generated pose using roll/pitch errors from BNO055.

`src/sensors.py` adds foot-load feedback through ADS1115 and FSR sensors. The walking engine uses this signal as a support-transition guard:

```text
if planned_support_load_ratio < threshold:
    hold support-shift frames
else:
    allow swing phase
```

This prevents the robot from lifting a foot before the other foot is carrying enough body weight.

Correct report wording:

```text
The feedback layer combines bounded IMU postural correction with FSR-based support-foot validation on top of the model-based gait planner.
```

## Get-Up

`src/getup.py` is not a LIPM/ZMP controller. It is an open-loop calibrated recovery sequence with smooth interpolation and PWM bounds. It should be reported as:

```text
An experimentally tuned finite-state recovery motion.
```

## Validation Command

Run:

```powershell
python tools\validate_algorithms.py
```

Expected output:

```text
algorithm validation passed: IK/FK endpoint, ZMP preview, get-up bounds
```

## References

- Kajita et al., "Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point", IEEE ICRA 2003, DOI `10.1109/ROBOT.2003.1241826`.
- The LIPM/ZMP equation and preview-control problem formulation are from the cart-table model in the Kajita paper.
- LIKO, "LiDAR, Inertial, and Kinematic Odometry for Bipedal Robots", is used as conceptual support for treating foot-contact/contact-state information as part of biped state estimation. The current project uses a simplified implementation with one FSR per foot rather than full force/torque sensing.
- Raspberry Pi 5 official power documentation is used to justify a dedicated 5.1V/5A computing power domain.
- Adafruit PCA9685 servo-driver documentation is used to justify separate high-current servo power, common ground, and bulk capacitance near the servo power rail.
- Adafruit BNO055 documentation is used to justify using onboard IMU fusion for application-focused closed-loop balance instead of adding a custom EKF in the first deployment.
- Adafruit ADS1115 documentation is used to justify the ADC stage between analog FSR sensors and Raspberry Pi.
- `pib.rocks`, Poppy Project, and Salvius are used as architecture references: Raspberry Pi handles high-level behavior, while actuator/sensor electronics remain modular and separately powered.
