from __future__ import annotations

from dataclasses import dataclass


PWM_PER_DEG = 2000.0 / 180.0


# --- Physical Robot Dimensions & Properties ---
ROBOT = {
    "com_height": 147.4,
    "half_hip": 28.0,
    "upper_leg": 80.0,
    "lower_leg": 75.0,
}

# --- Calibrated standing pulse widths ---
STANDING = {
    9: 1500,    # Left elbow
    10: 2450,   # Left upper arm down
    11: 1500,   # Left shoulder swing
    12: 1500,  # Left hip roll/abduction
    13: 1522,   # Left hip pitch
    14: 1500,   # Left knee
    15: 1500,   # Left ankle pitch
    16: 1500 ,  # Left ankle roll / foot
    17: 1500,  # Right ankle roll / foot
    18: 1500,   # Right ankle pitch
    19: 1500,   # Right knee
    20: 1478,   # Right hip pitch
    21: 1500,  # Right hip roll/abduction
    22: 1470,   # Right shoulder swing
    23: 500,    # Right upper arm down
    24: 1500,   # Right elbow
    25: 1500,   # Head
}

# --- Calibrated standing joint angles ---
STAND_ANG = {
    "R_hip_pitch": 18.0,
    "R_knee": 36.0,
    "R_ankle": 18.0,
    "R_hip_abduct": 4.0,
    "L_hip_pitch": 18.0,
    "L_knee": 36.0,
    "L_ankle": 18.0,
    "L_hip_abduct": 4.0,
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
    port: str = "/dev/serial/by-id/usb-RTrobot_RTrobot_Servo_Controller_56FF64483438-if00"
    baudrate: int = 115200
    csv: str = "out/log.csv"
    update_ms: int = 30
    stop_ms: int = 250

    # --- Manual gait: distances in mm, durations in seconds ---
    walk_step_length_mm: float = 24.0
    walk_turn_length_mm: float = 8.1
    walk_side_length_mm: float = 18.15
    walk_step_time_s: float = 1.725
    walk_side_step_time_s: float = 0.69
    walk_turn_step_time_s: float = 1.015
    walk_settle_time_s: float = 0.81
    side_swing_tempo: float = 3.375
    # Shared leg geometry/profile; autonomous modes use shorter, slower steps.
    walk_step_height_mm: float = 58.24
    walk_hip_out_deg: float = 3.0
    walk_crouch_depth_mm: float = 8.0
    walk_forward_lean_deg: float = 1.0
    walk_lift_start_phase: float = 0.30
    walk_swing_advance_end_phase: float = 0.60
    walk_lift_end_phase: float = 0.86
    zmp_support_ratio: float = 0.80
    ankle_roll_gain: float = -1.00
    walk_arm_forward_pwm: int = 167  # Nominal 15 degrees forward, held while walking
    auto_step_time_s: float = 1.26
    auto_settle_time_s: float = 1.47

    # --- Live Camera & Person Follow ---
    vision_camera_width: int = 480
    vision_camera_height: int = 360
    vision_fps: int = 12
    head_pan_pwm: int = 220
    head_pan_direction: float = 1.0

    # --- Web Control ---
    gait_dashboard_host: str = "0.0.0.0"
    gait_dashboard_port: int = 8765
    gait_dashboard_stream_hz: int = 12
    gait_dashboard_command_timeout_s: float = 0.6

    # --- IMU Gait Health ---
    gait_anomaly_enabled: bool = True
    gait_anomaly_model: str = "deploy/models/gait_anomaly.json"
    gait_anomaly_history: str = "out/gait_health_history.jsonl"
    gait_anomaly_window_s: float = 2.5
    gait_anomaly_min_samples: int = 30
    gait_anomaly_warning_windows: int = 2

    # --- Person Detection & Follow ---
    person_detect_prototxt: str = "../person_detect/MobileNetSSD_deploy.prototxt"
    person_detect_model: str = "../person_detect/MobileNetSSD_deploy.caffemodel"
    person_detect_confidence: float = 0.55
    person_detect_every_frames: int = 3
    person_detect_stable_frames: int = 3
    person_follow_turn_deadband: float = 0.10
    person_follow_step_length_mm: float = 8.64
    person_follow_turn_length_mm: float = 1.44
    person_follow_crawl_band_mm: int = 100  # Low-speed range above the stop distance; not a stop zone.
    person_follow_slow_range_mm: int = 700
    person_follow_tof_filter_alpha: float = 0.30
    person_camera_tof_height_mm: float = 130.0
    person_camera_hfov_deg: float = 53.5
    person_camera_vfov_deg: float = 41.41
    person_tof_fov_deg: float = 45.0
    person_tof_flip_vertical: bool = True

    # --- Stair Detection & Climbing ---
    stair_model: str = "deploy/models/stair_detector.onnx"
    stair_model_confidence: float = 0.55
    stair_model_iou_threshold: float = 0.45
    stair_model_input_size: int = 416
    stair_detect_every_frames: int = 3
    stair_detect_stable_frames: int = 4
    stair_camera_align_deadband: float = 0.12
    stair_approach_step_mm: float = 2.16
    stair_turn_step_mm: float = 0.40
    stair_default_riser_mm: float = 20.0
    stair_min_riser_mm: float = 15.0
    stair_max_riser_mm: float = 25.0
    stair_tread_depth_mm: float = 160.0
    stair_width_mm: float = 320.0
    stair_step_depth_mm: float = 120.0  # Maximum stride, not tread depth.
    stair_foot_clearance_mm: float = 18.0
    stair_crouch_depth_mm: float = 35.0
    stair_phase_shift_s: float = 1.20
    stair_phase_swing_s: float = 2.80
    stair_phase_transfer_s: float = 1.20
    stair_phase_settle_s: float = 1.50
    stair_step_pause_s: float = 0.70
    # Measure on the robot before enabling automatic foot placement.
    stair_geometry_calibrated: bool = False
    stair_foot_toe_mm: float = 0.0
    stair_foot_heel_mm: float = 0.0
    stair_foot_width_mm: float = 0.0
    stair_landing_margin_mm: float = 8.0
    stair_tof_forward_offset_mm: float = 0.0
    stair_tof_mount_height_mm: float = 220.0
    stair_tof_pitch_down_deg: float = 0.0  # Chest ToF faces forward; measure any actual tilt.
    stair_tof_vertical_fov_deg: float = 45.0
    stair_tof_flip_vertical: bool = True
    # --- Terrain IMU Balance ---
    terrain_balance_limit_deg: float = 8.0
    terrain_balance_deadband_deg: float = 0.35

    # --- Shared ToF stop distance: people and obstacles ---
    tof_obstacle_stop_mm: int = 100
    tof_obstacle_clear_margin_mm: int = 100
    tof_obstacle_stable_frames: int = 3

    # --- Dance ---
    dance_transition: float = 0.45
    dance_shoulder_pwm: int = 420
    dance_elbow_pwm: int = 420
    dance_lift_pwm: int = 820
    dance_head_pwm: int = 180

    # --- Manual Squat ---
    manual_squat_depth_mm: float = 75.0
    manual_squat_forward_mm: float = 0.0
    manual_squat_arm_forward_pwm: int = 650
    manual_squat_arm_raise_s: float = 0.23
    manual_squat_transition_s: float = 2.2

    # --- Getup ---
    getup_speed: float = 1.0

    # --- Balance ---
    imu_roll_sign: float = 1.0
    imu_pitch_sign: float = 1.0
    imu_yaw_sign: float = 1.0
    imu_vertical_mount: bool = True
    imu_board_face_sign: float = 1.0  # +Z/component side faces robot front; use -1 if it faces rear
    imu_reference_seconds: float = 1.5
    imu_reference_timeout_s: float = 8.0
    imu_reference_max_rms_deg: float = 2.0
    imu_min_gyro_cal: int = 2
    imu_min_accel_cal: int = 0
    fall_detection_enabled: bool = True
    fall_trigger_tilt_deg: float = 18.0
    fall_trigger_rate_deg_s: float = 70.0
    fall_hard_tilt_deg: float = 30.0
    fall_trigger_frames: int = 2
    fall_reset_tilt_deg: float = 8.0
    fall_reset_frames: int = 12
    fall_arm_forward_pwm: int = 1000  # Shoulder swing: nominal 90 degrees forward

    # --- Sensor Feedback ---
    sensor_port: str = "auto"
    sensor_baudrate: int = 115200
    sensor_timeout_s: float = 0.25
    sensor_depth_timeout_s: float = 0.65
    sensor_use_imu: bool = True  # Fall safety also requires IMU, even if this is False.
    sensor_use_foot_fsr: bool = True
    sensor_use_depth: bool = True
    foot_fsr_invert: bool = False
    foot_fsr_filter_alpha: float = 0.18
    foot_fsr_zero_raw: int = 0
    foot_fsr_full_raw: int = 4095
