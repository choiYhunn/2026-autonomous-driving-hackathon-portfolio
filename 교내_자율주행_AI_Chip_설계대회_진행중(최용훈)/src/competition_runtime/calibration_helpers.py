"""Full Team08 calibration workflow for CALIBRATION_TEAM08_FULL.ipynb.

Nothing moves on import.  Rear drive motors are never enabled by this module.
Steering motion occurs only inside explicitly called calibration methods, always
at 100% duty and with short, bounded pulses followed by steer_off() in finally.
"""
from __future__ import annotations

import csv
import io
import json
import math
import time
import zipfile
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import ipywidgets as widgets
from IPython.display import display, clear_output

from pynq import MMIO
from pynq_dpu import DpuOverlay

from AutoLab_lib import init
from config import (
    DRIVE_MOTOR_ADDRESSES,
    ADDRESS_RANGE,
    DRIVE_DPU_BIT,
    PARKING_BIT,
    XMODEL,
    CLASSES_PATH,
    ANCHORS,
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    X1,
)
from calibration_io import BASE_DIR, save_calibration
from motor_controller import MotorController
from image_processor import ImageProcessor


ADC_LABELS = ("right_full", "center", "left_full")
PULSE_START_LABELS = ("right_half", "center", "left_half")
PULSE_DURATIONS_MS = (8, 12, 16, 24, 32, 48)
PULSE_REPEATS = 3
LANE_LABELS = ("in_touch", "in_safe", "ideal", "out_safe", "out_touch")


class CalibrationSession:
    def __init__(self, output_dir: str = "calibration_output") -> None:
        init()
        self.output_dir = BASE_DIR / output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        # Calibration belongs to the normal driving profile.  Load its DPU first
        # so the 0x00A0... motor addresses are valid before MMIO construction.
        print(f"Loading verified driving DPU for calibration: {DRIVE_DPU_BIT}")
        self.overlay = DpuOverlay(DRIVE_DPU_BIT)
        self.overlay.load_model(XMODEL)
        self.motors = {
            name: MMIO(address, ADDRESS_RANGE)
            for name, address in DRIVE_MOTOR_ADDRESSES.items()
        }
        self.motor = MotorController(self.motors)
        self.motor.stop_all()

        self.image_processor: Optional[ImageProcessor] = ImageProcessor(
            self.overlay.runner, CLASSES_PATH, ANCHORS
        )
        self.cap = None

        self.adc_samples: Dict[str, List[int]] = {}
        self.lane_samples: Dict[str, List[float]] = {}
        self.pulse_tests: List[dict] = []
        self.direction_checks: List[dict] = []
        self.positioning_tests: List[dict] = []
        self.timing_samples_ms: List[float] = []
        self.pulse_model_result: Optional[dict] = None
        self.positioning_validation_result: Optional[dict] = None
        self.manual_vehicle = {
            "wheelbase_m": 0.64,
            "track_width_m": 0.50,
            "tire_width_m": 0.09,
            "speed_at_100_duty_mps": 0.90,
        }
        self.result: Dict[str, object] = {}
        print("Calibration session ready. ALL MOTORS ARE OFF.")
        print("Steering calibration always uses PWM duty 100 with bounded pulses.")
        print(f"Driving DPU bit: {DRIVE_DPU_BIT} (exists={Path(DRIVE_DPU_BIT).exists()})")
        print(f"XMODEL: {XMODEL} (exists={Path(XMODEL).exists()})")

    # ------------------------------------------------------------------
    # Generic helpers
    # ------------------------------------------------------------------
    def close(self) -> None:
        self.motor.stop_all()
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        print("Calibration resources released; motors OFF.")

    @staticmethod
    def _summary(values: Sequence[float]) -> dict:
        if not values:
            return {}
        arr = np.asarray(values, dtype=float)
        return {
            "n": int(arr.size),
            "median": float(np.median(arr)),
            "mean": float(np.mean(arr)),
            "std": float(np.std(arr)),
            "min": float(np.min(arr)),
            "max": float(np.max(arr)),
            "p05": float(np.percentile(arr, 5)),
            "p10": float(np.percentile(arr, 10)),
            "p90": float(np.percentile(arr, 90)),
            "p95": float(np.percentile(arr, 95)),
        }

    def _adc_median(self, n: int = 11, interval_s: float = 0.002) -> int:
        values: List[int] = []
        for _ in range(int(n)):
            value = self.motor.read_adc()
            if self.motor.valid_adc_value(value):
                values.append(int(value))
            time.sleep(interval_s)
        if len(values) < max(3, int(n * 0.7)):
            raise RuntimeError(f"ADC read unstable: {len(values)}/{n} valid")
        return int(round(float(np.median(values))))

    def emergency_stop(self) -> None:
        self.motor.stop_all()
        print("EMERGENCY STOP: all motor valid signals OFF")

    def check_paths(self) -> dict:
        report = {
            "drive_dpu_bit": {"path": DRIVE_DPU_BIT, "exists": Path(DRIVE_DPU_BIT).exists()},
            "parking_bit": {"path": PARKING_BIT, "exists": Path(PARKING_BIT).exists()},
            "xmodel": {"path": XMODEL, "exists": Path(XMODEL).exists()},
            "classes": {"path": CLASSES_PATH, "exists": Path(CLASSES_PATH).exists()},
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return report

    # ------------------------------------------------------------------
    # ADC three-anchor calibration (HALF targets derived)
    # ------------------------------------------------------------------
    def capture_adc(self, label: str, n: int = 250, interval_s: float = 0.01) -> dict:
        if label not in ADC_LABELS:
            raise ValueError(f"label must be one of {ADC_LABELS}")
        self.motor.stop_all()
        values: List[int] = []
        for _ in range(int(n)):
            value = self.motor.read_adc()
            if self.motor.valid_adc_value(value):
                values.append(int(value))
            time.sleep(interval_s)
        if len(values) < max(50, int(n * 0.85)):
            raise RuntimeError(f"too many invalid ADC samples: {len(values)}/{n}")
        self.adc_samples[label] = values
        summary = self._summary(values)
        print(label, json.dumps(summary, ensure_ascii=False, indent=2))
        return summary

    def adc_report(self) -> dict:
        missing = [name for name in ADC_LABELS if name not in self.adc_samples]
        if missing:
            raise RuntimeError(f"missing ADC captures: {missing}")
        report = {name: self._summary(self.adc_samples[name]) for name in ADC_LABELS}
        rf = float(report["right_full"]["median"])
        center = float(report["center"]["median"])
        lf = float(report["left_full"]["median"])
        if not (rf < center < lf):
            raise RuntimeError(
                "ADC anchor order invalid. Expected right_full < center < left_full, "
                f"got {rf}, {center}, {lf}"
            )

        def derived_summary(value: float) -> dict:
            return {
                "n": 0, "median": value, "mean": value, "std": 0.0,
                "min": value, "max": value, "p05": value, "p10": value,
                "p90": value, "p95": value, "derived": True,
            }

        # HALF positions are never measured by eye.  They are numerical targets
        # derived from the three physical anchors and are reached automatically.
        report["right_half"] = derived_summary((rf + center) / 2.0)
        report["left_half"] = derived_summary((center + lf) / 2.0)
        center_std = report["center"]["std"]
        center_range = report["center"]["p95"] - report["center"]["p05"]
        report["noise"] = {
            "center_std_adc": center_std,
            "center_p05_p95_width_adc": center_range,
            "initial_effective_movement_threshold_adc": max(3.0, 4.0 * center_std),
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return report

    def apply_adc_to_session(self) -> dict:
        report = self.adc_report()
        self.motor.adc_right_full = int(round(report["right_full"]["median"]))
        self.motor.adc_right_half = int(round(report["right_half"]["median"]))
        self.motor.adc_center = int(round(report["center"]["median"]))
        self.motor.adc_left_half = int(round(report["left_half"]["median"]))
        self.motor.adc_left_full = int(round(report["left_full"]["median"]))
        self.motor.resistance_most_right = self.motor.adc_right_full
        self.motor.resistance_center = self.motor.adc_center
        self.motor.resistance_most_left = self.motor.adc_left_full
        print("Three measured ADC anchors plus derived HALF targets applied to this session.")
        return report

    def adc_capture_widget(self):
        out = widgets.Output()
        buttons = []
        descriptions = {
            "right_full": "1 Right FULL",
            "center": "2 CENTER",
            "left_full": "3 Left FULL",
        }

        def callback(label: str):
            def run(_):
                with out:
                    clear_output(wait=True)
                    try:
                        self.capture_adc(label)
                    except Exception as exc:
                        self.emergency_stop()
                        print("ERROR:", exc)
            return run

        for label in ADC_LABELS:
            btn = widgets.Button(description=descriptions[label])
            btn.on_click(callback(label))
            buttons.append(btn)
        stop = widgets.Button(description="STOP ALL", button_style="danger")
        stop.on_click(lambda _: self.emergency_stop())
        box = widgets.VBox([
            widgets.HTML(
                "<b>앞바퀴를 띄우고 세 물리 기준점만 측정합니다.</b><br>"
                "CENTER는 긴 자/각재를 앞바퀴에 대고 차체 중심선과 평행하게 맞춘 뒤 측정하세요.<br>"
                "RIGHT/LEFT HALF는 눈대중으로 측정하지 않고 코드가 ADC 중간값으로 자동 생성합니다."
            ),
            widgets.HBox(buttons + [stop]),
            out,
        ])
        display(box)
        return box

    # ------------------------------------------------------------------
    # Safe raw 100%-duty positioning used before pulse model exists
    # ------------------------------------------------------------------
    def _raw_pulse(self, direction: str, pulse_ms: float) -> None:
        try:
            if direction == "left":
                self.motor.steer_left(100)
            elif direction == "right":
                self.motor.steer_right(100)
            else:
                raise ValueError(direction)
            time.sleep(max(0.0, float(pulse_ms)) / 1000.0)
        finally:
            self.motor.steer_off()

    def safe_position_to_adc(
        self,
        target_adc: float,
        timeout_s: float = 8.0,
        tolerance_adc: Optional[float] = None,
    ) -> dict:
        """Conservative pre-model positioner using 8/12/20 ms pulses at 100%."""
        report = self.adc_report()
        noise = float(report["center"]["std"])
        tolerance = max(8.0, 3.0 * noise) if tolerance_adc is None else float(tolerance_adc)
        low_limit = report["right_full"]["median"] - 10
        high_limit = report["left_full"]["median"] + 10
        start = time.monotonic()
        attempts = 0
        sign_changes = 0
        last_sign = 0
        try:
            while time.monotonic() - start < timeout_s:
                adc = self._adc_median(7, 0.001)
                if not (low_limit <= adc <= high_limit):
                    raise RuntimeError(f"ADC crossed calibrated limit: {adc}")
                error = float(target_adc) - adc
                if abs(error) <= tolerance:
                    self.motor.steer_off()
                    return {
                        "success": True,
                        "target_adc": float(target_adc),
                        "final_adc": adc,
                        "final_error_adc": error,
                        "attempts": attempts,
                        "sign_changes": sign_changes,
                    }
                sign = 1 if error > 0 else -1
                if last_sign and sign != last_sign:
                    sign_changes += 1
                    self.motor.steer_off()
                    time.sleep(0.10)
                last_sign = sign
                magnitude = abs(error)
                pulse_ms = 20 if magnitude > 120 else 12 if magnitude > 55 else 8
                self._raw_pulse("left" if error > 0 else "right", pulse_ms)
                attempts += 1
                time.sleep(0.045)
            final_adc = self._adc_median(7, 0.001)
            return {
                "success": False,
                "target_adc": float(target_adc),
                "final_adc": final_adc,
                "final_error_adc": float(target_adc) - final_adc,
                "attempts": attempts,
                "sign_changes": sign_changes,
            }
        finally:
            self.motor.steer_off()

    # ------------------------------------------------------------------
    # Direction sanity check
    # ------------------------------------------------------------------
    def verify_steering_directions(self, pulse_ms: float = 12.0) -> dict:
        self.apply_adc_to_session()
        center = float(self.adc_report()["center"]["median"])
        threshold = max(3.0, 4.0 * float(self.adc_report()["center"]["std"]))
        results = []
        for direction, expected_sign in (("left", 1), ("right", -1)):
            pos = self.safe_position_to_adc(center)
            if not pos["success"]:
                raise RuntimeError(f"could not center before {direction} direction check")
            time.sleep(0.15)
            before = self._adc_median(11, 0.001)
            self._raw_pulse(direction, pulse_ms)
            time.sleep(0.20)
            after = self._adc_median(11, 0.001)
            delta = after - before
            passed = (delta * expected_sign) >= threshold
            record = {
                "direction": direction,
                "pulse_ms": pulse_ms,
                "adc_before": before,
                "adc_after": after,
                "adc_delta": delta,
                "expected_sign": expected_sign,
                "threshold_adc": threshold,
                "passed": passed,
            }
            results.append(record)
            print(json.dumps(record, ensure_ascii=False, indent=2))
            self.safe_position_to_adc(center)
        self.direction_checks = results
        overall = all(r["passed"] for r in results)
        if not overall:
            raise RuntimeError("Steering direction check FAILED. Do not continue.")
        print("PASS: motor_5 increases ADC (left), motor_4 decreases ADC (right).")
        return {"passed": overall, "checks": results}

    # ------------------------------------------------------------------
    # Full pulse + coast matrix
    # ------------------------------------------------------------------
    def _pulse_observation(
        self,
        start_label: str,
        direction: str,
        pulse_ms: float,
        repeat_index: int,
    ) -> dict:
        report = self.adc_report()
        target = float(report[start_label]["median"])
        positioned = self.safe_position_to_adc(target)
        if not positioned["success"]:
            raise RuntimeError(f"could not position at {start_label}: {positioned}")
        time.sleep(0.15)
        before = self._adc_median(11, 0.001)
        low_limit = report["right_full"]["median"] - 10
        high_limit = report["left_full"]["median"] + 10

        off_time = None
        try:
            if direction == "left":
                self.motor.steer_left(100)
            elif direction == "right":
                self.motor.steer_right(100)
            else:
                raise ValueError(direction)
            time.sleep(float(pulse_ms) / 1000.0)
        finally:
            self.motor.steer_off()
            off_time = time.monotonic()

        immediate = self._adc_median(5, 0.001)
        observations = {"0": immediate}
        for delay_ms in (20, 50, 100, 200):
            remaining = delay_ms / 1000.0 - (time.monotonic() - off_time)
            if remaining > 0:
                time.sleep(remaining)
            observations[str(delay_ms)] = self._adc_median(5, 0.001)
        final_adc = observations["200"]
        if not (low_limit <= final_adc <= high_limit):
            raise RuntimeError(f"pulse crossed calibrated mechanical range: {final_adc}")

        record = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "start_label": start_label,
            "direction": direction,
            "duty_percent": 100.0,
            "pulse_ms": float(pulse_ms),
            "repeat_index": int(repeat_index),
            "adc_before": int(before),
            "adc_immediate": int(immediate),
            "adc_20ms": int(observations["20"]),
            "adc_50ms": int(observations["50"]),
            "adc_100ms": int(observations["100"]),
            "adc_200ms": int(final_adc),
            "delta_immediate_adc": int(immediate - before),
            "delta_total_adc": int(final_adc - before),
            "coast_adc": int(final_adc - immediate),
            "positioning_start_error_adc": float(positioned["final_error_adc"]),
        }
        return record

    def run_full_pulse_matrix(
        self,
        durations_ms: Sequence[float] = PULSE_DURATIONS_MS,
        repeats: int = PULSE_REPEATS,
    ) -> dict:
        if not self.direction_checks or not all(x.get("passed") for x in self.direction_checks):
            raise RuntimeError("Run verify_steering_directions() and obtain PASS first")
        self.apply_adc_to_session()
        total = len(PULSE_START_LABELS) * 2 * len(durations_ms) * int(repeats)
        print(f"Starting FULL matrix: {total} bounded pulses, duty=100%.")
        print("Front wheels must remain lifted. Rear drive motors remain OFF.")
        completed = 0
        try:
            for start_label in PULSE_START_LABELS:
                for direction in ("left", "right"):
                    for pulse_ms in durations_ms:
                        for repeat_index in range(int(repeats)):
                            rec = self._pulse_observation(
                                start_label, direction, float(pulse_ms), repeat_index
                            )
                            self.pulse_tests.append(rec)
                            completed += 1
                            print(
                                f"[{completed:03d}/{total}] {start_label:10s} {direction:5s} "
                                f"{pulse_ms:>4.0f}ms total={rec['delta_total_adc']:+4d} "
                                f"coast={rec['coast_adc']:+3d}"
                            )
        except KeyboardInterrupt:
            print("Interrupted by user.")
            raise
        finally:
            self.motor.steer_off()
            self.motor.stop_drive()
        result = self.build_pulse_model()
        print("FULL pulse matrix complete.")
        return result

    @staticmethod
    def _region_for_start_label(start_label: str) -> str:
        return {
            "right_half": "right_region",
            "center": "center_region",
            "left_half": "left_region",
        }[start_label]

    def build_pulse_model(self) -> dict:
        if not self.pulse_tests:
            raise RuntimeError("pulse matrix is empty")
        adc_report = self.adc_report()
        noise_std = float(adc_report["center"]["std"])
        effective_threshold = max(3.0, 4.0 * noise_std)
        model: Dict[str, Dict[str, List[dict]]] = {
            d: {r: [] for r in ("right_region", "center_region", "left_region")}
            for d in ("left", "right")
        }
        effective_movements = []

        for direction in ("left", "right"):
            expected_sign = 1 if direction == "left" else -1
            for start_label in PULSE_START_LABELS:
                region = self._region_for_start_label(start_label)
                for duration in sorted({float(x["pulse_ms"]) for x in self.pulse_tests}):
                    rows = [
                        x for x in self.pulse_tests
                        if x["direction"] == direction
                        and x["start_label"] == start_label
                        and float(x["pulse_ms"]) == duration
                    ]
                    if not rows:
                        continue
                    totals = np.asarray([float(x["delta_total_adc"]) for x in rows])
                    coasts = np.asarray([float(x["coast_adc"]) for x in rows])
                    sign_rate = float(np.mean(np.sign(totals) == expected_sign))
                    entry = {
                        "pulse_ms": duration,
                        "median_total_adc": float(np.median(totals)),
                        "p90_abs_total_adc": float(np.percentile(np.abs(totals), 90)),
                        "p95_abs_total_adc": float(np.percentile(np.abs(totals), 95)),
                        "median_coast_adc": float(np.median(coasts)),
                        "p90_abs_coast_adc": float(np.percentile(np.abs(coasts), 90)),
                        "sign_consistency": sign_rate,
                        "n": len(rows),
                    }
                    if sign_rate < 0.80:
                        raise RuntimeError(
                            f"unstable pulse direction: {direction}/{start_label}/{duration}ms "
                            f"sign consistency={sign_rate:.2f}"
                        )
                    model[direction][region].append(entry)
                    if entry["p90_abs_total_adc"] >= effective_threshold:
                        effective_movements.append(entry["p90_abs_total_adc"])

        if not effective_movements:
            raise RuntimeError("No pulse duration produced motion above ADC noise")
        min_effective = float(min(effective_movements))
        stop_band = max(4.0, math.ceil(3.0 * noise_std), math.ceil(0.30 * min_effective))
        start_band = max(stop_band + 2.0, math.ceil(stop_band + min_effective / 0.80))

        # Estimate how long coast remains materially different from the 200 ms value.
        settle_threshold = max(2.0, 3.0 * noise_std)
        settle_times = []
        for row in self.pulse_tests:
            final_value = float(row["adc_200ms"])
            selected = 200
            for delay in (20, 50, 100, 200):
                if abs(float(row[f"adc_{delay}ms"]) - final_value) <= settle_threshold:
                    selected = delay
                    break
            settle_times.append(selected)
        reversal_settle_ms = float(np.clip(np.percentile(settle_times, 90), 50, 200))

        result = {
            "duty_percent": 100.0,
            "pulse_durations_ms": sorted({float(x["pulse_ms"]) for x in self.pulse_tests}),
            "pulse_model": model,
            "prediction_safety_factor": 0.80,
            "reversal_settle_ms": reversal_settle_ms,
            "minimum_command_interval_ms": 5.0,
            "effective_movement_threshold_adc": effective_threshold,
            "minimum_effective_p90_movement_adc": min_effective,
            "recommended_stop_band_adc": float(stop_band),
            "recommended_start_band_adc": float(start_band),
            "matrix_n": len(self.pulse_tests),
        }
        self.pulse_model_result = result
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return result

    # ------------------------------------------------------------------
    # Closed-loop target positioning validation
    # ------------------------------------------------------------------
    def _adc_region_from_report(self, adc: float) -> str:
        r = self.adc_report()
        right_boundary = (r["right_half"]["median"] + r["center"]["median"]) / 2.0
        left_boundary = (r["center"]["median"] + r["left_half"]["median"]) / 2.0
        if adc < right_boundary:
            return "right_region"
        if adc > left_boundary:
            return "left_region"
        return "center_region"

    def _select_model_pulse(self, direction: str, current_adc: float, error_adc: float) -> Optional[float]:
        if self.pulse_model_result is None:
            self.build_pulse_model()
        model = self.pulse_model_result
        stop_band = float(model["recommended_stop_band_adc"])
        allowed = max(0.0, abs(error_adc) - stop_band) * float(model["prediction_safety_factor"])
        region = self._adc_region_from_report(current_adc)
        entries = model["pulse_model"][direction][region]
        candidates = [e for e in entries if float(e["p90_abs_total_adc"]) <= allowed]
        if not candidates:
            return None
        return float(max(candidates, key=lambda e: float(e["p90_abs_total_adc"]))["pulse_ms"])

    def position_with_calibrated_model(
        self,
        target_adc: float,
        timeout_s: float = 6.0,
    ) -> dict:
        if self.pulse_model_result is None:
            self.build_pulse_model()
        model = self.pulse_model_result
        stop_band = float(model["recommended_stop_band_adc"])
        start_band = float(model["recommended_start_band_adc"])
        reversal_s = float(model["reversal_settle_ms"]) / 1000.0
        start = time.monotonic()
        attempts = 0
        reversals = 0
        crossings = 0
        last_error = None
        last_direction = None
        history = []
        try:
            while time.monotonic() - start < timeout_s:
                adc = self._adc_median(7, 0.001)
                error = float(target_adc) - adc
                history.append({"adc": adc, "error_adc": error})
                if last_error is not None and error * last_error < 0:
                    crossings += 1
                last_error = error
                if abs(error) <= stop_band:
                    return {
                        "success": True,
                        "target_adc": float(target_adc),
                        "final_adc": adc,
                        "final_error_adc": error,
                        "attempts": attempts,
                        "reversals": reversals,
                        "target_crossings": crossings,
                        "history": history,
                    }
                direction = "left" if error > 0 else "right"
                if last_direction and direction != last_direction:
                    reversals += 1
                    self.motor.steer_off()
                    time.sleep(reversal_s)
                pulse_ms = self._select_model_pulse(direction, adc, error)
                if pulse_ms is None:
                    # No pulse is conservatively safe. Accept only if within the
                    # calibrated practical start band; otherwise fail visibly.
                    return {
                        "success": abs(error) <= start_band,
                        "target_adc": float(target_adc),
                        "final_adc": adc,
                        "final_error_adc": error,
                        "attempts": attempts,
                        "reversals": reversals,
                        "target_crossings": crossings,
                        "history": history,
                        "reason": "no_safe_pulse",
                    }
                self._raw_pulse(direction, pulse_ms)
                attempts += 1
                last_direction = direction
                time.sleep(0.035)
            adc = self._adc_median(7, 0.001)
            return {
                "success": False,
                "target_adc": float(target_adc),
                "final_adc": adc,
                "final_error_adc": float(target_adc) - adc,
                "attempts": attempts,
                "reversals": reversals,
                "target_crossings": crossings,
                "history": history,
                "reason": "timeout",
            }
        finally:
            self.motor.steer_off()

    def _steer_to_adc_from_report(self, steer: float) -> float:
        r = self.adc_report()
        table = [
            (-7.0, r["left_full"]["median"]),
            (-3.5, r["left_half"]["median"]),
            (0.0, r["center"]["median"]),
            (3.5, r["right_half"]["median"]),
            (7.0, r["right_full"]["median"]),
        ]
        s = max(-7.0, min(7.0, float(steer)))
        for (s0, a0), (s1, a1) in zip(table, table[1:]):
            if s <= s1:
                ratio = (s - s0) / (s1 - s0)
                return a0 + ratio * (a1 - a0)
        return table[-1][1]

    def validate_target_positioning(self, repeats: int = 3) -> dict:
        if self.pulse_model_result is None:
            self.build_pulse_model()
        center = float(self.adc_report()["center"]["median"])
        # Covers the entire production lane-control range: MAX_STEER = +/-5.
        sweep_units = (0.0, 1.75, 3.5, 5.0, 0.0, -1.75, -3.5, -5.0, 0.0)
        records = []
        for repeat in range(int(repeats)):
            init_pos = self.safe_position_to_adc(center)
            if not init_pos["success"]:
                raise RuntimeError("could not initialize center before validation")
            for step, steering_unit in enumerate(sweep_units):
                target_adc = self._steer_to_adc_from_report(steering_unit)
                result = self.position_with_calibrated_model(target_adc)
                record = {
                    "repeat": repeat,
                    "step": step,
                    "target_steer_unit": steering_unit,
                    **{k: v for k, v in result.items() if k != "history"},
                }
                records.append(record)
                print(
                    f"validation r{repeat+1}/{repeats} step{step}: steer={steering_unit:+.2f} "
                    f"targetADC={target_adc:.1f} final={result['final_adc']} "
                    f"error={result['final_error_adc']:+.1f} success={result['success']}"
                )
                if not result["success"]:
                    self.positioning_tests = records
                    raise RuntimeError(f"positioning validation failed: {record}")
        self.positioning_tests = records
        errors = np.asarray([abs(float(x["final_error_adc"])) for x in records])
        start_band = float(self.pulse_model_result["recommended_start_band_adc"])
        passed = (
            all(bool(x["success"]) for x in records)
            and float(np.percentile(errors, 95)) <= start_band
            and float(np.max(errors)) <= 1.5 * start_band
        )
        report = {
            "passed": bool(passed),
            "n": len(records),
            "mean_abs_error_adc": float(np.mean(errors)),
            "p95_abs_error_adc": float(np.percentile(errors, 95)),
            "max_abs_error_adc": float(np.max(errors)),
            "start_band_adc": start_band,
            "total_target_crossings": int(sum(int(x["target_crossings"]) for x in records)),
            "total_reversals": int(sum(int(x["reversals"]) for x in records)),
            "records": records,
        }
        self.positioning_validation_result = report
        print(json.dumps(report, ensure_ascii=False, indent=2))
        if not passed:
            raise RuntimeError("Positioning validation FAILED. Do not export/run autonomous.")
        print("PASS: calibrated 100%-duty pulse controller reached all target positions.")
        return report

    # ------------------------------------------------------------------
    # Vision and physical lane-limit calibration
    # ------------------------------------------------------------------
    def start_vision(self) -> None:
        if self.image_processor is None:
            raise RuntimeError("driving DPU/image processor was not initialized")
        if self.cap is None:
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            if not self.cap.isOpened():
                self.cap = None
                raise RuntimeError("camera open failed")
        print("DPU/camera ready. Drive motors remain OFF.")

    def _read_frame(self):
        if self.cap is None:
            self.start_vision()
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("camera frame read failed")
        return frame

    def preview_lane(self):
        self.motor.stop_all()
        frame = self._read_frame()
        lane, diag, ms, _ = self.image_processor.infer_lane_center(frame)
        ok, jpg = cv2.imencode(".jpg", diag)
        if ok:
            display(widgets.Image(value=jpg.tobytes(), format="jpg"))
        print(f"raw lane_center={lane}, DPU={ms:.1f} ms")
        return lane

    def capture_lane(self, label: str, n: int = 80, interval_s: float = 0.01) -> dict:
        if label not in LANE_LABELS:
            raise ValueError(f"label must be one of {LANE_LABELS}")
        self.motor.stop_all()
        self.start_vision()
        values: List[float] = []
        last_diag = None
        times = []
        for _ in range(int(n)):
            frame = self._read_frame()
            lane, diag, ms, _ = self.image_processor.infer_lane_center(frame)
            times.append(ms)
            if lane is not None:
                values.append(float(lane))
                last_diag = diag
            time.sleep(interval_s)
        if len(values) < max(30, int(n * 0.75)):
            raise RuntimeError(f"lane detection insufficient: {len(values)}/{n}")
        self.lane_samples[label] = values
        self.timing_samples_ms.extend(times)
        if last_diag is not None:
            cv2.imwrite(str(self.output_dir / f"lane_{label}.jpg"), last_diag)
        summary = self._summary(values)
        summary["detection_rate"] = len(values) / float(n)
        summary["dpu_ms_median"] = float(np.median(times))
        print(label, json.dumps(summary, ensure_ascii=False, indent=2))
        return summary

    def lane_capture_widget(self):
        out = widgets.Output()
        preview = widgets.Button(description="Preview", button_style="info")
        buttons = []
        descriptions = {
            "in_touch": "1 IN TOUCH",
            "in_safe": "2 IN SAFE",
            "ideal": "3 IDEAL",
            "out_safe": "4 OUT SAFE",
            "out_touch": "5 OUT TOUCH",
        }

        def run(fn):
            def callback(_):
                with out:
                    clear_output(wait=True)
                    try:
                        fn()
                    except Exception as exc:
                        self.emergency_stop()
                        print("ERROR:", exc)
            return callback

        preview.on_click(run(self.preview_lane))
        for label in LANE_LABELS:
            btn = widgets.Button(description=descriptions[label])
            btn.on_click(run(lambda label=label: self.capture_lane(label)))
            buttons.append(btn)
        close_btn = widgets.Button(description="Close camera", button_style="warning")
        close_btn.on_click(run(self.close_camera))
        help_text = widgets.HTML(
            "<b>직선 2차선, 차량과 차선을 완전히 평행하게 유지하고 손으로만 이동하세요.</b><br>"
            "IN TOUCH: 안쪽 타이어가 중앙 점선에 막 닿기 시작하는 위치.<br>"
            "IN SAFE: IN TOUCH에서 차로 안쪽으로 30 mm 이동한 안전 위치.<br>"
            "IDEAL: 평소 직선 주행 목표 위치.<br>"
            "OUT SAFE: OUT TOUCH에서 차로 안쪽으로 30 mm 이동한 안전 위치.<br>"
            "OUT TOUCH: 바깥 타이어가 바깥 실선에 막 닿기 시작하는 위치.<br>"
            "각 위치에서 차체 yaw가 바뀌면 측정이 무효입니다. 카메라 마운트도 움직이지 마세요."
        )
        box = widgets.VBox([
            help_text,
            widgets.HBox([preview] + buttons[:3]),
            widgets.HBox(buttons[3:] + [close_btn]),
            out,
        ])
        display(box)
        return box

    def close_camera(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        print("camera closed")

    def lane_report(self) -> dict:
        missing = [name for name in LANE_LABELS if name not in self.lane_samples]
        if missing:
            raise RuntimeError(f"missing lane captures: {missing}")
        summaries = {name: self._summary(self.lane_samples[name]) for name in LANE_LABELS}
        offsets = {name: float(X1 - summaries[name]["median"]) for name in LANE_LABELS}
        touch_min = min(offsets["in_touch"], offsets["out_touch"])
        touch_max = max(offsets["in_touch"], offsets["out_touch"])
        safe_min = min(offsets["in_safe"], offsets["out_safe"])
        safe_max = max(offsets["in_safe"], offsets["out_safe"])
        center = offsets["ideal"]
        if not (touch_min < safe_min < safe_max < touch_max):
            raise RuntimeError(
                "Lane limit order invalid. Touch limits must enclose safe limits. "
                f"touch=[{touch_min:.2f},{touch_max:.2f}], safe=[{safe_min:.2f},{safe_max:.2f}]"
            )
        if not (safe_min <= center <= safe_max):
            raise RuntimeError(
                f"IDEAL offset {center:.2f} is outside safe range [{safe_min:.2f}, {safe_max:.2f}]"
            )
        available = min(center - safe_min, safe_max - center)
        recommended_span = max(0.0, min(2.0, available * 0.5))
        report = {
            "captures": summaries,
            "equivalent_offsets_px": offsets,
            "recommended": {
                "offset_center_px": round(center, 3),
                "offset_min_touch_px": round(touch_min, 3),
                "offset_max_touch_px": round(touch_max, 3),
                "offset_min_safe_px": round(safe_min, 3),
                "offset_max_safe_px": round(safe_max, 3),
                "offset_span_px": 0.0,
                "offset_span_after_track_test_px": round(recommended_span, 3),
            },
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return report

    # ------------------------------------------------------------------
    # Optional dimensions / speed
    # ------------------------------------------------------------------
    def set_vehicle_measurements(
        self,
        wheelbase_m: Optional[float] = None,
        track_width_m: Optional[float] = None,
        tire_width_m: Optional[float] = None,
        measured_distance_m: Optional[float] = None,
        measured_time_s: Optional[float] = None,
    ) -> dict:
        if wheelbase_m is not None:
            self.manual_vehicle["wheelbase_m"] = float(wheelbase_m)
        if track_width_m is not None:
            self.manual_vehicle["track_width_m"] = float(track_width_m)
        if tire_width_m is not None:
            self.manual_vehicle["tire_width_m"] = float(tire_width_m)
        if measured_distance_m and measured_time_s and measured_time_s > 0:
            self.manual_vehicle["speed_at_100_duty_mps"] = float(measured_distance_m / measured_time_s)
        print(json.dumps(self.manual_vehicle, ensure_ascii=False, indent=2))
        return dict(self.manual_vehicle)

    # ------------------------------------------------------------------
    # Result validation/export
    # ------------------------------------------------------------------
    def pre_export_check(self) -> dict:
        checks = {
            "adc_three_anchors": all(x in self.adc_samples for x in ADC_LABELS),
            "direction_check": bool(self.direction_checks) and all(x.get("passed") for x in self.direction_checks),
            "pulse_matrix": len(self.pulse_tests) >= len(PULSE_START_LABELS) * 2 * len(PULSE_DURATIONS_MS) * PULSE_REPEATS,
            "pulse_model": self.pulse_model_result is not None,
            "positioning_validation": bool(self.positioning_validation_result and self.positioning_validation_result.get("passed")),
            "lane_five_positions": all(x in self.lane_samples for x in LANE_LABELS),
        }
        checks["all_passed"] = all(checks.values())
        print(json.dumps(checks, ensure_ascii=False, indent=2))
        return checks

    def build_result(self) -> dict:
        checks = self.pre_export_check()
        if not checks["all_passed"]:
            raise RuntimeError("Calibration incomplete. Complete every required step before export.")
        adc = self.adc_report()
        lane = self.lane_report()
        pulse = self.pulse_model_result or self.build_pulse_model()
        validation = self.positioning_validation_result
        dpu_median = float(np.median(self.timing_samples_ms)) if self.timing_samples_ms else 65.0
        result = {
            "schema_version": 2,
            "calibration_complete": True,
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "adc": {
                "right_full": int(round(adc["right_full"]["median"])),
                "right_half": int(round(adc["right_half"]["median"])),
                "center": int(round(adc["center"]["median"])),
                "left_half": int(round(adc["left_half"]["median"])),
                "left_full": int(round(adc["left_full"]["median"])),
                "center_std": round(adc["center"]["std"], 4),
                "stop_band_adc": pulse["recommended_stop_band_adc"],
                "start_band_adc": pulse["recommended_start_band_adc"],
            },
            "steering": {
                "duty_percent": 100.0,
                "pulse_durations_ms": pulse["pulse_durations_ms"],
                "pulse_model": pulse["pulse_model"],
                "prediction_safety_factor": pulse["prediction_safety_factor"],
                "reversal_settle_ms": pulse["reversal_settle_ms"],
                "minimum_command_interval_ms": pulse["minimum_command_interval_ms"],
                "positioning_validation_passed": True,
                "positioning_validation_summary": {
                    k: v for k, v in validation.items() if k != "records"
                },
            },
            "lane": {
                "x1": X1,
                "base_y": 240,
                "point_detection_height": 70,
                "offset_center_px": lane["recommended"]["offset_center_px"],
                "offset_span_px": 0.0,
                "offset_span_after_track_test_px": lane["recommended"]["offset_span_after_track_test_px"],
                "curve_scale_px": 35.0,
                "offset_min_safe_px": lane["recommended"]["offset_min_safe_px"],
                "offset_max_safe_px": lane["recommended"]["offset_max_safe_px"],
                "offset_min_touch_px": lane["recommended"]["offset_min_touch_px"],
                "offset_max_touch_px": lane["recommended"]["offset_max_touch_px"],
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
                "camera_dpu_latency_s": round(dpu_median / 1000.0, 5),
                "controller_fps": 15.0,
            },
            "vehicle": dict(self.manual_vehicle),
            "raw_summary": {
                "checks": checks,
                "adc": adc,
                "lane": lane,
                "pulse_model_summary": {
                    k: v for k, v in pulse.items() if k != "pulse_model"
                },
                "positioning_validation": {
                    k: v for k, v in validation.items() if k != "records"
                },
                "direction_checks": self.direction_checks,
            },
        }
        self.result = result
        return result

    def _write_adc_csv(self) -> Path:
        path = self.output_dir / "adc_samples.csv"
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["label", "sample_index", "adc"])
            for label, values in self.adc_samples.items():
                for i, value in enumerate(values):
                    writer.writerow([label, i, value])
        return path

    def _write_lane_csv(self) -> Path:
        path = self.output_dir / "lane_samples.csv"
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["label", "sample_index", "lane_center_px"])
            for label, values in self.lane_samples.items():
                for i, value in enumerate(values):
                    writer.writerow([label, i, value])
        return path

    @staticmethod
    def _write_dict_csv(path: Path, rows: List[dict]) -> Path:
        if not rows:
            path.write_text("", encoding="utf-8")
            return path
        fields = []
        for row in rows:
            for key in row:
                if key not in fields and not isinstance(row[key], (dict, list)):
                    fields.append(key)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)
        return path

    def _write_report_txt(self, result: dict) -> Path:
        validation = result["steering"]["positioning_validation_summary"]
        lane = result["lane"]
        path = self.output_dir / "CALIBRATION_REPORT.txt"
        text = f"""TEAM08 FULL CALIBRATION REPORT
================================
Created: {result['created_at']}
Overall: PASS

ADC three measured anchors + two derived half targets
- right_full : {result['adc']['right_full']}
- right_half : {result['adc']['right_half']} (derived)
- center     : {result['adc']['center']}
- left_half  : {result['adc']['left_half']} (derived)
- left_full  : {result['adc']['left_full']}
- stop/start band: {result['adc']['stop_band_adc']} / {result['adc']['start_band_adc']} ADC

Steering pulse controller
- duty: 100%
- pulse durations: {result['steering']['pulse_durations_ms']} ms
- reversal settle: {result['steering']['reversal_settle_ms']} ms
- positioning validation: PASS
- validation p95 abs error: {validation['p95_abs_error_adc']:.2f} ADC
- validation max abs error: {validation['max_abs_error_adc']:.2f} ADC

Lane limits
- touch range: [{lane['offset_min_touch_px']:.2f}, {lane['offset_max_touch_px']:.2f}] px
- safe range : [{lane['offset_min_safe_px']:.2f}, {lane['offset_max_safe_px']:.2f}] px
- ideal offset: {lane['offset_center_px']:.2f} px
- production offset_span: 0.0 px (enable 1 then 2 only after track test)

Next
1. Restart the Python kernel so config.py reloads calibration_result.json.
2. Open RUN_TEAM08.ipynb.
3. First test straight with offset_span_px=0.
4. Upload TEAM08_CALIBRATION_UPLOAD.zip for final review/tuning.
"""
        path.write_text(text, encoding="utf-8")
        return path

    def export(self) -> Path:
        result = self.build_result()
        target = save_calibration(result)
        files = [
            target,
            self._write_adc_csv(),
            self._write_lane_csv(),
            self._write_dict_csv(self.output_dir / "steering_pulse_matrix.csv", self.pulse_tests),
            self._write_dict_csv(self.output_dir / "positioning_validation.csv", self.positioning_tests),
            self._write_dict_csv(self.output_dir / "direction_checks.csv", self.direction_checks),
        ]
        full_json = self.output_dir / "calibration_result_full.json"
        full_json.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        files.append(full_json)
        report_path = self._write_report_txt(result)
        files.append(report_path)
        files.extend(sorted(self.output_dir.glob("lane_*.jpg")))

        upload_zip = self.output_dir / "TEAM08_CALIBRATION_UPLOAD.zip"
        with zipfile.ZipFile(upload_zip, "w", compression=zipfile.ZIP_DEFLATED) as zf:
            for path in files:
                if path.exists():
                    zf.write(path, arcname=path.name)
        print(f"Saved production calibration: {target}")
        print(f"Saved upload package: {upload_zip}")
        print("RESTART KERNEL before running RUN_TEAM08.ipynb.")
        return upload_zip
