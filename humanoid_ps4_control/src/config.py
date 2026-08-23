from __future__ import annotations

from dataclasses import dataclass


PWM_PER_DEG = 2000.0 / 180.0
STANCE_HIP_OUT_DEG = 4.0
STANCE_ANKLE_COMP_DEG = 0.0


# --- Physical Robot Dimensions & Properties ---
ROBOT = {
    "com_height": 147.4,
    "half_hip": 28.0,
    "upper_leg": 80.0,
    "lower_leg": 75.0,
}

# --- Default Gait Parameters ---
GAIT = {
    "zmp_support_ratio": 0.65,
    "ankle_roll_gain": -1.00,
    "command_deadzone": 0.02,
    "arm_swing_pwm": 240,
    "arm_right_dir": 1,
    "arm_left_dir": -1,
    "arm_smooth_tau": 0.12,
    "arm_min_pwm": 18,
    "arm_quantum_pwm": 6,
    "max_side_step_len": 38.0,
    "max_turn_step_len": 7.0,
}

# --- Calibrated standing pulse widths ---
STANDING = {
    9: 1500,    # Left elbow
    10: 2450,   # Left upper arm down
    11: 1500,   # Left shoulder swing
    12: round(1500 + STANCE_HIP_OUT_DEG * PWM_PER_DEG),  # Left hip roll/abduction
    13: 1500,   # Left hip pitch
    14: 1500,   # Left knee
    15: 1500,   # Left ankle pitch
    16: round(1500 + STANCE_ANKLE_COMP_DEG * PWM_PER_DEG),  # Left ankle roll / foot
    17: round(1500 - STANCE_ANKLE_COMP_DEG * PWM_PER_DEG),  # Right ankle roll / foot
    18: 1500,   # Right ankle pitch
    19: 1500,   # Right knee
    20: 1500,   # Right hip pitch
    21: round(1500 - STANCE_HIP_OUT_DEG * PWM_PER_DEG),  # Right hip roll/abduction
    22: 1470,   # Right shoulder swing
    23: 500,    # Right upper arm down
    24: 1500,   # Right elbow
    25: 1500,   # Head
}

# --- Calibrated standing joint angles ---
STAND_ANG = {
    "hip_roll": STANCE_ANKLE_COMP_DEG,
    "R_hip_pitch": 18.0,
    "R_knee": 36.0,
    "R_ankle": 18.0,
    "R_hip_abduct": STANCE_HIP_OUT_DEG,
    "L_hip_pitch": 18.0,
    "L_knee": 36.0,
    "L_ankle": 18.0,
    "L_hip_abduct": STANCE_HIP_OUT_DEG,
}

# --- Direction configuration per servo ---
DIR = {
    12: +1,
    13: +1,
    14: +1,
    15: +1,
    16: +1,
    17: -1,
    18: -1,
    19: -1,
    20: -1,
    21: -1,
}

@dataclass
class Config:
    # --- Hardware ---
    backend: str = "serial"
    port: str = "/dev/ttyACM0"
    baudrate: int = 115200
    csv: str = "out/log.csv"
    update_ms: int = 30
    stop_ms: int = 250

    # --- Walking Engine (Linked to GAIT values by default) ---
    walk_speed: float = 0.45
    turn_speed: float = 0.20
    side_speed: float = 0.50
    max_turn_step_len: float = GAIT["max_turn_step_len"]
    max_side_step_len: float = GAIT["max_side_step_len"]
    walk_step_length_mm: float = 42.0
    walk_step_height_mm: float = 60.0
    walk_crouch_depth_mm: float = 12.0
    walk_lift_start_phase: float = 0.24
    walk_swing_advance_end_phase: float = 0.60
    walk_lift_end_phase: float = 1.0
    walk_landing_roll_release_start: float = 0.42
    walk_crouch_transition_s: float = 0.45
    walk_max_leg_pwm_per_s: float = 1100.0
    t_step: float = 1.15
    t_dbl: float = 0.12
    zmp_support_ratio: float = GAIT["zmp_support_ratio"]
    ankle_roll_gain: float = GAIT["ankle_roll_gain"]
    command_rate_limit: float = 40.0
    one_foot_swing_knee_pwm: int = 700
    one_foot_arm_lift_pwm: int = 950
    one_foot_ramp_s: float = 0.75

    # --- Arms (Linked to GAIT values by default) ---
    arm_swing_pwm: int = GAIT["arm_swing_pwm"]
    arm_right_dir: int = GAIT["arm_right_dir"]
    arm_left_dir: int = GAIT["arm_left_dir"]
    arm_smooth_tau: float = GAIT["arm_smooth_tau"]
    arm_min_pwm: int = GAIT["arm_min_pwm"]
    arm_quantum_pwm: int = GAIT["arm_quantum_pwm"]

    # --- Live Camera & Person Follow ---
    vision_camera_width: int = 480
    vision_camera_height: int = 360
    vision_fps: int = 12
    head_pan_pwm: int = 220
    head_pan_rate_pwm_s: float = 700.0
    head_turn_lead_s: float = 0.30
    head_pan_direction: float = 1.0

    # --- Web Control ---
    gait_dashboard_host: str = "0.0.0.0"
    gait_dashboard_port: int = 8765
    gait_dashboard_stream_hz: int = 12
    gait_dashboard_history_s: int = 120
    gait_dashboard_command_timeout_s: float = 0.6

    # --- Person Detection & Follow ---
    person_detect_prototxt: str = "../person_detect/MobileNetSSD_deploy.prototxt"
    person_detect_model: str = "../person_detect/MobileNetSSD_deploy.caffemodel"
    person_detect_confidence: float = 0.55
    person_detect_every_frames: int = 3
    person_detect_stable_frames: int = 3
    person_follow_lost_timeout_s: float = 1.0
    person_follow_turn_deadband: float = 0.10
    person_follow_stop_height_ratio: float = 0.58
    person_follow_speed: float = 0.18
    person_follow_turn_speed: float = 0.12
    person_follow_target_distance_mm: int = 700
    person_follow_distance_deadband_mm: int = 100
    person_follow_slow_range_mm: int = 700
    person_follow_tof_filter_alpha: float = 0.30

    # --- Adaptive Squat ---
    squat_min_depth_mm: float = 12.0
    squat_max_depth_mm: float = 40.0
    squat_depth_rate_mm_s: float = 35.0
    squat_max_pwm_per_frame: float = 28.0

    # --- Terrain IMU Balance ---
    terrain_emergency_tilt_deg: float = 12.0
    terrain_balance_limit_deg: float = 5.0
    terrain_balance_deadband_deg: float = 0.6

    # --- ToF Obstacle Guard ---
    tof_obstacle_stop_mm: int = 350
    tof_obstacle_clear_margin_mm: int = 100
    tof_obstacle_stable_frames: int = 3

    # --- Dance ---
    dance_period: float = 2.4
    dance_transition: float = 0.45
    dance_shoulder_pwm: int = 420
    dance_elbow_pwm: int = 260
    dance_lift_pwm: int = 820
    dance_head_pwm: int = 180
    dance_smooth_tau: float = 0.08
    dance_max_pwm_per_sec: float = 2200.0
    dance_min_step_pwm: int = 18

    # --- Getup ---
    getup_mode: str = "back"
    getup_speed: float = 0.7

    # --- Balance ---
    imu_balance: bool = True
    imu_roll_sign: float = 1.0
    imu_pitch_sign: float = 1.0
    imu_yaw_sign: float = 1.0
    imu_vertical_mount: bool = True
    imu_board_face_sign: float = 1.0  # +Z/component side faces robot front; use -1 if it faces rear
    balance_limit_deg: float = 6.0
    balance_deadband_deg: float = 0.4
    imu_reference_seconds: float = 1.5
    imu_reference_timeout_s: float = 8.0
    imu_reference_max_rms_deg: float = 2.0
    imu_min_gyro_cal: int = 2
    imu_min_accel_cal: int = 0
    push_recovery_enabled: bool = True
    push_recovery_warning_tilt_deg: float = 3.0
    push_recovery_tilt_deg: float = 5.0
    push_recovery_safe_lower_tilt_deg: float = 9.0
    push_recovery_rate_deg_s: float = 28.0
    push_recovery_settle_tilt_deg: float = 1.4
    push_recovery_step_forward_cmd: float = 0.10
    push_recovery_step_side_cmd: float = 0.08
    push_recovery_step_time_s: float = 0.80
    push_recovery_step_height_mm: float = 8.0
    push_recovery_timeout_s: float = 3.0
    push_recovery_counter_lean_s: float = 0.40
    push_recovery_counter_lean_deg: float = 1.5
    push_recovery_lower_rate_pwm_s: float = 300.0
    fall_detection_enabled: bool = True
    fall_trigger_tilt_deg: float = 18.0
    fall_trigger_rate_deg_s: float = 70.0
    fall_hard_tilt_deg: float = 30.0
    fall_trigger_frames: int = 2
    fall_reset_tilt_deg: float = 8.0
    fall_arm_forward_pwm: int = 600

    # --- Sensor Feedback ---
    sensor_feedback: bool = True
    sensor_port: str = "auto"
    sensor_baudrate: int = 115200
    sensor_timeout_s: float = 0.25
    sensor_depth_timeout_s: float = 0.65
    sensor_use_imu: bool = True
    sensor_use_foot_fsr: bool = True
    sensor_use_depth: bool = True
    foot_fsr_invert: bool = False
    foot_fsr_filter_alpha: float = 0.18
    foot_fsr_zero_raw: int = 0
    foot_fsr_full_raw: int = 4095
