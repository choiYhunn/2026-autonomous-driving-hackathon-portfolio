"""Team08 production configuration (schema v2).

CALIBRATION_TEAM08_FULL.ipynb creates calibration_result.json. Autonomous mode
is intentionally locked until a schema-v2 calibration file exists.
"""
from __future__ import annotations

import os
from pathlib import Path
import numpy as np

from calibration_io import BASE_DIR, CALIBRATION_FILE, load_calibration

# ===== Hardware profiles =====
# Normal driving DPU bitstream: original motor map (0x00A0...).
DRIVE_MOTOR_ADDRESSES = {
    "motor_0": 0x00A0000000,  # right rear backward
    "motor_1": 0x00A0010000,  # right rear forward
    "motor_2": 0x00A0020000,  # left rear forward
    "motor_3": 0x00A0030000,  # left rear backward
    "motor_4": 0x00A0040000,  # steering right
    "motor_5": 0x00A0050000,  # steering left
}

# Parking custom bitstream: PWM + ultrasonic map (0x800...).
PARKING_MOTOR_ADDRESSES = {
    "motor_0": 0x80010000,  # right rear backward
    "motor_1": 0x80020000,  # right rear forward
    "motor_2": 0x80030000,  # left rear forward
    "motor_3": 0x80040000,  # left rear backward
    "motor_4": 0x80050000,  # steering right
    "motor_5": 0x80060000,  # steering left
}
PARKING_ULTRASONIC_ADDRESSES = {
    "ultrasonic_0": 0x80070000,  # right front
    "ultrasonic_1": 0x80080000,  # right rear
    "ultrasonic_2": 0x80090000,  # left
}
ULTRASONIC_SENSOR_MAP = {
    "right_front": "ultrasonic_0",
    "right_rear": "ultrasonic_1",
    "left": "ultrasonic_2",
}

# Backward-compatible names always mean the normal driving profile.
MOTOR_ADDRESSES = DRIVE_MOTOR_ADDRESSES
ULTRASONIC_ADDRESSES = PARKING_ULTRASONIC_ADDRESSES
ADDRESS_RANGE = 0x10000
PWM_SIZE = 600600  # 2 ms period

# ===== Camera / model =====
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240


def _resolve_path(env_name: str, candidates: list[str]) -> str:
    value = os.environ.get(env_name)
    if value:
        return value
    search = []
    for relative in candidates:
        search.extend([BASE_DIR / relative, BASE_DIR.parent / relative, Path.cwd() / relative])
    for candidate in search:
        if candidate.exists():
            return str(candidate.resolve())
    return str((BASE_DIR / candidates[0]).resolve())


DRIVE_DPU_BIT = _resolve_path("DRIVE_DPU_BIT_PATH", [
    "../dpu/dpu.bit",
    "dpu/dpu.bit",
])
PARKING_BIT = _resolve_path("PARKING_BIT_PATH", [
    "../debugging/CYH/dpu_cyh_2/dpu.bit",
    "debugging/CYH/dpu_cyh_2/dpu.bit",
])
# Calibration and legacy code use the verified normal driving DPU.
DPU_BIT = DRIVE_DPU_BIT
XMODEL = _resolve_path("XMODEL_PATH", [
    "../xmodel/tiny-yolov3_256.xmodel",
    "xmodel/tiny-yolov3_256.xmodel",
])
CLASSES_PATH = _resolve_path("CLASSES_PATH", [
    "../xmodel/lane_class.txt",
    "xmodel/lane_class.txt",
])
ANCHOR_LIST = [10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319]
ANCHORS = np.array(ANCHOR_LIST).reshape(-1, 2)
classes_path = CLASSES_PATH
anchors = ANCHORS

# Fallbacks permit import/manual/parking, but autonomous mode remains locked.
_DEFAULT_PULSE_MODEL = {
    direction: {
        region: [
            {"pulse_ms": 8.0, "median_total_adc": sign * 5.0, "p90_abs_total_adc": 7.0, "median_coast_adc": sign * 1.0, "n": 0},
            {"pulse_ms": 12.0, "median_total_adc": sign * 8.0, "p90_abs_total_adc": 11.0, "median_coast_adc": sign * 1.5, "n": 0},
            {"pulse_ms": 16.0, "median_total_adc": sign * 12.0, "p90_abs_total_adc": 16.0, "median_coast_adc": sign * 2.0, "n": 0},
            {"pulse_ms": 24.0, "median_total_adc": sign * 18.0, "p90_abs_total_adc": 24.0, "median_coast_adc": sign * 3.0, "n": 0},
            {"pulse_ms": 32.0, "median_total_adc": sign * 25.0, "p90_abs_total_adc": 33.0, "median_coast_adc": sign * 4.0, "n": 0},
            {"pulse_ms": 48.0, "median_total_adc": sign * 36.0, "p90_abs_total_adc": 47.0, "median_coast_adc": sign * 6.0, "n": 0},
        ]
        for region in ("right_region", "center_region", "left_region")
    }
    for direction, sign in (("left", 1.0), ("right", -1.0))
}

_DEFAULT_CALIBRATION = {
    "schema_version": 2,
    "calibration_complete": False,
    "adc": {
        "right_full": 1920,
        "right_half": 2055,
        "center": 2191,
        "left_half": 2345,
        "left_full": 2500,
        "center_std": 3.0,
        "stop_band_adc": 6.0,
        "start_band_adc": 15.0,
    },
    "steering": {
        "duty_percent": 100.0,
        "pulse_durations_ms": [8, 12, 16, 24, 32, 48],
        "pulse_model": _DEFAULT_PULSE_MODEL,
        "prediction_safety_factor": 0.80,
        "reversal_settle_ms": 90.0,
        "minimum_command_interval_ms": 5.0,
        "positioning_validation_passed": False,
    },
    "lane": {
        "x1": 164,
        "base_y": 240,
        "point_detection_height": 70,
        "offset_center_px": 25.0,
        "offset_span_px": 0.0,
        "curve_scale_px": 35.0,
        "offset_min_safe_px": 18.0,
        "offset_max_safe_px": 32.0,
        "offset_min_touch_px": 15.0,
        "offset_max_touch_px": 35.0,
        "target_max_step_px": 6.0,
        "target_filter_alpha": 0.35,
        "initial_angle_deg": 90.0,
        "lane_lost_stop_frames": 8,
    },
    "drive": {
        "speed_percent": 100.0,
        "full_at_deg": 20.0,
        "max_steer": 5.0,
        "invalid_adc_stop_count": 5,
    },
    "timing": {
        "camera_dpu_latency_s": 0.065,
        "controller_fps": 15.0,
    },
    "vehicle": {
        "wheelbase_m": 0.64,
        "track_width_m": 0.50,
        "tire_width_m": 0.09,
        "speed_at_100_duty_mps": 0.90,
    },
}

CAL = load_calibration(_DEFAULT_CALIBRATION)
CALIBRATION_READY = bool(
    CALIBRATION_FILE.exists()
    and int(CAL.get("schema_version", 0)) >= 2
    and CAL.get("calibration_complete", False)
    and CAL.get("steering", {}).get("positioning_validation_passed", False)
)

# ===== ADC position table =====
ADC_RIGHT_FULL = int(CAL["adc"]["right_full"])
ADC_RIGHT_HALF = int(CAL["adc"]["right_half"])
ADC_CENTER = int(CAL["adc"]["center"])
ADC_LEFT_HALF = int(CAL["adc"]["left_half"])
ADC_LEFT_FULL = int(CAL["adc"]["left_full"])
ADC_STOP_BAND = float(CAL["adc"]["stop_band_adc"])
ADC_START_BAND = float(CAL["adc"]["start_band_adc"])

# Backward-compatible aliases.
RESIST_RIGHT = ADC_RIGHT_FULL
RESIST_CENTER = ADC_CENTER
RESIST_LEFT = ADC_LEFT_FULL
STEER_ON_THRESHOLD = ADC_START_BAND
STEER_OFF_THRESHOLD = ADC_STOP_BAND

# ===== Steering pulse controller =====
STEERING_DUTY_PERCENT = float(CAL["steering"]["duty_percent"])
STEERING_PULSE_DURATIONS_MS = [float(x) for x in CAL["steering"]["pulse_durations_ms"]]
STEERING_PULSE_MODEL = CAL["steering"]["pulse_model"]
STEERING_PREDICTION_SAFETY_FACTOR = float(CAL["steering"]["prediction_safety_factor"])
STEERING_REVERSAL_SETTLE_MS = float(CAL["steering"]["reversal_settle_ms"])
STEERING_MIN_COMMAND_INTERVAL_MS = float(CAL["steering"]["minimum_command_interval_ms"])
DIRECTION_CHANGE_PAUSE_SEC = 0.15

# Parking/manual aliases. User confirmed PWM below 100 is unstable.
STEERING_SPEED_LEFT = 100.0
STEERING_SPEED_RIGHT = 100.0
STEERING_HOLD_DUTY = 0.0
STEER_TOLERANCE = ADC_START_BAND

# ===== Drive =====
DRIVE_SPEED = float(CAL["drive"]["speed_percent"])
FULL_AT_DEG = float(CAL["drive"]["full_at_deg"])
MAX_STEER = float(CAL["drive"]["max_steer"])
INVALID_ADC_STOP_COUNT = int(CAL["drive"]["invalid_adc_stop_count"])

# ===== Perception =====
X1 = int(CAL["lane"]["x1"])
BASE_Y = int(CAL["lane"]["base_y"])
POINT_DETECTION_HEIGHT = int(CAL["lane"]["point_detection_height"])
OFFSET_CENTER = float(CAL["lane"]["offset_center_px"])
OFFSET_SPAN = float(CAL["lane"]["offset_span_px"])
CURVE_SCALE = float(CAL["lane"]["curve_scale_px"])
OFFSET_MIN_SAFE = float(CAL["lane"]["offset_min_safe_px"])
OFFSET_MAX_SAFE = float(CAL["lane"]["offset_max_safe_px"])
OFFSET_MIN_TOUCH = float(CAL["lane"]["offset_min_touch_px"])
OFFSET_MAX_TOUCH = float(CAL["lane"]["offset_max_touch_px"])
TARGET_MAX_STEP = float(CAL["lane"]["target_max_step_px"])
TARGET_FILTER_ALPHA = float(CAL["lane"]["target_filter_alpha"])
INITIAL_ANGLE = float(CAL["lane"]["initial_angle_deg"])
LANE_LOST_STOP_FRAMES = int(CAL["lane"]["lane_lost_stop_frames"])

# ===== Parking tuning =====
PARKING = {
    "front_near_min_cm": 30,
    "front_near_max_cm": 80,
    "front_far_min_cm": 80,
    "front_far_max_cm": 120,
    "front_confirm_count": 2,
    "front_max_spread_cm": 12,
    "rear_detect_max_cm": 120,
    "rear_confirm_count": 2,
    "rear_max_spread_cm": 12,
    "approach_sample_sec": 0.04,
    "approach_max_sec": 8.0,
    "approach_speed": 100,
    "pre_straight_near_sec": 1.0,
    "pre_straight_far_sec": 1.0,
    "left_entry_sec": 1.3,
    "right_reverse_sec": 2.15,
    "straight_reverse_sec": 1.35,
    "park_wait_sec": 3.5,
    "exit_center_forward_sec": 0.50,
    "exit_right_forward_sec": 3.50,
    "final_straight_sec": 5.0,
    "forward_speed": 100,
    "reverse_speed": 100,
    "exit_speed": 100,
    "steer_left_speed": 100,
    "steer_right_speed": 100,
    "center_speed": 100,
    "center_tolerance": max(5, int(round(ADC_STOP_BAND))),
    "center_confirm_count": 3,
    "center_settle_sec": max(0.04, STEERING_REVERSAL_SETTLE_MS / 1000.0),
    "steer_limit_tolerance": 20,
}


def print_active_config() -> None:
    source = str(CALIBRATION_FILE) if CALIBRATION_FILE.exists() else "fallback defaults"
    print(f"[CONFIG] source: {source}")
    print(f"[CONFIG] autonomous calibration ready: {CALIBRATION_READY}")
    print(f"[CONFIG] drive bit: {DRIVE_DPU_BIT}")
    print(f"[CONFIG] parking bit: {PARKING_BIT}")
    print(
        "[CONFIG] ADC RF/RH/C/LH/LF="
        f"{ADC_RIGHT_FULL}/{ADC_RIGHT_HALF}/{ADC_CENTER}/{ADC_LEFT_HALF}/{ADC_LEFT_FULL}; "
        f"bands stop/start={ADC_STOP_BAND:.1f}/{ADC_START_BAND:.1f}"
    )
    print(
        f"[CONFIG] offset={OFFSET_CENTER:.1f}, span={OFFSET_SPAN:.1f}, "
        f"safe=[{OFFSET_MIN_SAFE:.1f}, {OFFSET_MAX_SAFE:.1f}]"
    )
