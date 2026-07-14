from __future__ import annotations

from dataclasses import dataclass


# --- Physical Robot Dimensions & Properties ---
ROBOT = {
    "com_height": 147.4,
    "half_hip": 28.0,
    "upper_leg": 80.0,
    "lower_leg": 75.0,
    "step_height": 34.0,
}

# --- Default Gait Parameters ---
GAIT = {
    "zmp_support_ratio": 0.96,
    "hip_abduct_gain": 0.34,
    "swing_hip_roll_scale": 0.0,
    "ankle_roll_gain": -0.30,
    "swing_ankle_roll_scale": 0.0,
    "step_x_ratio": 1.72,
    "thigh_lift_forward_mm": 0.0,
    "left_swing_x_scale": 1.0,
    "left_step_height_scale": 1.0,
    "landing_gap_mm": 68.0,
    "right_swing_x_scale": 1.0,
    "right_step_height_scale": 1.0,
    "lift_start_phase": 0.00,
    "swing_advance_end_phase": 0.62,
    "lift_end_phase": 1.0,
    "landing_roll_release_start": 0.42,
    "command_deadzone": 0.02,
    "arm_swing_pwm": 200,
    "arm_right_dir": 1,
    "arm_left_dir": -1,
    "arm_elbow_ratio": 0.0,
    "arm_lift_ratio": 0.0,
    "arm_smooth_tau": 0.08,
    "arm_min_pwm": 30,
    "arm_quantum_pwm": 10,
    "max_side_step_len": 38.0,
    "max_turn_step_len": 7.0,
    "stop_extra_steps": 4,
}

# --- Calibrated standing pulse widths ---
STANDING = {
    1: 1500,    # Right ankle roll
    2: 1500,    # Right ankle pitch
    3: 1500,    # Right knee
    4: 1500,    # Right hip pitch, standing forward bias
    5: 1500,    # Right hip roll/abduction
    6: 1500,    # Right elbow
    7: 500,     # Right upper arm down
    8: 1470,    # Right shoulder swing
    16: 1500,   # Head
    17: 1500,   # Left shoulder swing
    18: 2450,   # Left upper arm down
    19: 1500,   # Left elbow
    20: 1500,   # Left hip roll/abduction
    21: 1500,   # Left hip pitch, standing forward bias
    22: 1500,   # Left knee
    23: 1500,   # Left ankle pitch
    24: 1500,   # Left ankle roll
}

# --- Calibrated standing joint angles ---
STAND_ANG = {
    "hip_roll": 0.0,
    "R_hip_pitch": 18.0,
    "R_knee": 36.0,
    "R_ankle": 18.0,
    "R_hip_abduct": 0.0,
    "L_hip_pitch": 18.0,
    "L_knee": 36.0,
    "L_ankle": 18.0,
    "L_hip_abduct": 0.0,
}

# --- Direction configuration per servo ---
DIR = {
    1: -1,
    2: -1,
    3: -1,
    4: -1,
    5: -1,
    20: +1,
    21: +1,
    22: +1,
    23: +1,
    24: +1,
}

PWM_PER_DEG = 2000.0 / 180.0


@dataclass
class Config:
    # --- Run Mode ---
    getup: bool = False

    # --- Hardware ---
    backend: str = "serial"
    port: str = "/dev/ttyACM0"
    baudrate: int = 115200
    csv: str = "out/log.csv"
    update_ms: int = 30
    stop_ms: int = 250

    # --- Walking Engine (Linked to GAIT values by default) ---
    walk_speed: float = 0.55
    turn_speed: float = 0.25
    side_speed: float = 0.45
    max_step_len: float = 34.0
    max_turn_step_len: float = GAIT["max_turn_step_len"]
    max_side_step_len: float = GAIT["max_side_step_len"]
    step_height: float = 30.0
    t_step: float = 1.08
    t_dbl: float = 0.08
    zmp_support_ratio: float = GAIT["zmp_support_ratio"]
    hip_abduct_gain: float = GAIT["hip_abduct_gain"]
    ankle_roll_gain: float = GAIT["ankle_roll_gain"]
    step_x_ratio: float = GAIT["step_x_ratio"]
    thigh_lift_forward_mm: float = GAIT["thigh_lift_forward_mm"]
    left_swing_x_scale: float = GAIT["left_swing_x_scale"]
    left_step_height_scale: float = GAIT["left_step_height_scale"]
    right_swing_x_scale: float = GAIT["right_swing_x_scale"]
    right_step_height_scale: float = GAIT["right_step_height_scale"]
    landing_gap_mm: float = GAIT["landing_gap_mm"]
    lift_start_phase: float = GAIT["lift_start_phase"]
    swing_advance_end_phase: float = GAIT["swing_advance_end_phase"]
    lift_end_phase: float = GAIT["lift_end_phase"]
    landing_roll_release_start: float = GAIT["landing_roll_release_start"]
    command_rate_limit: float = 24.0
    swing_hip_roll_scale: float = GAIT["swing_hip_roll_scale"]
    swing_ankle_roll_scale: float = GAIT["swing_ankle_roll_scale"]
    single_support_lift_height: float = 82.0
    single_support_arm_pwm: int = 180
    single_support_ramp_s: float = 0.8

    # --- Arms (Linked to GAIT values by default) ---
    arm_swing_pwm: int = GAIT["arm_swing_pwm"]
    arm_right_dir: int = GAIT["arm_right_dir"]
    arm_left_dir: int = GAIT["arm_left_dir"]
    arm_elbow_ratio: float = GAIT["arm_elbow_ratio"]
    arm_lift_ratio: float = GAIT["arm_lift_ratio"]
    arm_smooth_tau: float = GAIT["arm_smooth_tau"]
    arm_min_pwm: int = GAIT["arm_min_pwm"]
    arm_quantum_pwm: int = GAIT["arm_quantum_pwm"]

    # --- Control Mode ---
    input_mode: str = "auto"
    joystick_index: int = 0
    input_deadzone: float = 0.08
    ps4_forward_axis: int = 1
    ps4_forward_sign: float = -1.0
    ps4_turn_axis: int = 0
    ps4_turn_sign: float = -1.0

    # --- Camera Mimic ---
    vision_camera_width: int = 480
    vision_camera_height: int = 360
    vision_fps: int = 12
    vision_threads: int = 3
    vision_confidence: float = 0.30
    vision_model_path: str = "models/movenet_lightning_int8.tflite"
    vision_min_body_scale: float = 0.10
    vision_lost_timeout_s: float = 0.35
    vision_smooth_tau_s: float = 0.12
    vision_max_pwm_per_s: float = 900.0
    vision_lift_pwm: int = 820
    vision_shoulder_pwm: int = 420
    vision_elbow_pwm: int = 260
    vision_head_pwm: int = 180
    vision_squat_deg: float = 16.0
    vision_leg_lift_threshold_ratio: float = 0.30
    vision_leg_lift_height_mm: float = 34.0
    vision_fsr_stable_frames: int = 3

    # --- Autonomous Terrain Vision ---
    terrain_camera_width: int = 480
    terrain_camera_height: int = 360
    terrain_camera_fps: int = 10
    terrain_stable_frames: int = 6
    terrain_unknown_frames: int = 3
    terrain_calibration_frames: int = 24
    terrain_roi_top_ratio: float = 0.28
    terrain_horizon_delta_ratio: float = 0.055
    terrain_horizon_up_sign: float = 1.0
    terrain_min_confidence: float = 0.58
    terrain_t_step: float = 1.55
    terrain_t_dbl: float = 0.12
    terrain_ramp_step_elevation_mm: float = 3.0
    terrain_stair_rise_mm: float = 12.0
    terrain_stair_tread_mm: float = 62.0
    terrain_max_step_elevation_mm: float = 18.0
    terrain_allow_stairs_down: bool = True
    terrain_emergency_tilt_deg: float = 12.0
    terrain_fsr_invalid_frames: int = 12
    terrain_touchdown_min_load: float = 0.04
    terrain_touchdown_invalid_frames: int = 8

    # --- Dance ---
    dance_period: float = 2.4
    dance_transition: float = 0.45
    dance_shoulder_pwm: int = 420
    dance_elbow_pwm: int = 260
    dance_lift_pwm: int = 820
    dance_head_pwm: int = 180
    dance_head_speed: float = 1.0
    dance_smooth_tau: float = 0.08
    dance_max_pwm_per_sec: float = 2200.0
    dance_min_step_pwm: int = 18

    # --- Getup ---
    getup_mode: str = "back"
    getup_speed: float = 0.7
    pre_stand_ms: int = 0
    final_stand_ms: int = 1200
    getup_print_every: int = 20

    # --- Balance ---
    imu_balance: bool = True
    imu_roll_sign: float = 1.0
    imu_pitch_sign: float = 1.0
    imu_yaw_sign: float = 1.0
    imu_vertical_mount: bool = True
    imu_board_face_sign: float = 1.0  # +Z/component side faces robot front; use -1 if it faces rear
    balance_limit_deg: float = 2.0
    balance_deadband_deg: float = 0.8
    imu_reference_seconds: float = 1.5
    imu_reference_timeout_s: float = 8.0
    imu_reference_max_rms_deg: float = 2.0
    imu_min_gyro_cal: int = 2
    imu_min_accel_cal: int = 0

    # --- Sensor Feedback ---
    sensor_feedback: bool = True
    sensor_port: str = "/dev/ttyUSB0"
    sensor_baudrate: int = 115200
    sensor_timeout_s: float = 0.25
    sensor_use_imu: bool = True
    sensor_use_fsr: bool = True
    sensor_debug: bool = False
    fsr_invert: bool = False
    fsr_filter_alpha: float = 0.18
    fsr_left_zero_raw: int = 0
    fsr_left_full_raw: int = 4095
    fsr_right_zero_raw: int = 0
    fsr_right_full_raw: int = 4095
    fsr_min_total_load: float = 0.08
    fsr_support_ratio: float = 0.60
