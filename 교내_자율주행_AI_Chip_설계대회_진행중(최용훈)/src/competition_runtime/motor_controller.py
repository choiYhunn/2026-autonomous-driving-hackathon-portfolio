"""Team08 motor controller using calibrated 100%-duty steering pulses.

The steering motor is never driven below 100% duty.  Steering magnitude is
controlled by pulse duration, and the calibrated ADC pulse model predicts how
far each pulse plus mechanical coast will move the steering mechanism.
"""
from __future__ import annotations

import time
from typing import Callable, Dict, Optional, Tuple

try:
    import spidev  # type: ignore
except ImportError:  # desktop syntax/unit tests
    spidev = None

from config import (
    PWM_SIZE,
    DRIVE_SPEED,
    FULL_AT_DEG,
    MAX_STEER,
    DIRECTION_CHANGE_PAUSE_SEC,
    ADC_RIGHT_FULL,
    ADC_RIGHT_HALF,
    ADC_CENTER,
    ADC_LEFT_HALF,
    ADC_LEFT_FULL,
    ADC_STOP_BAND,
    ADC_START_BAND,
    STEERING_DUTY_PERCENT,
    STEERING_PULSE_MODEL,
    STEERING_PREDICTION_SAFETY_FACTOR,
    STEERING_REVERSAL_SETTLE_MS,
    STEERING_MIN_COMMAND_INTERVAL_MS,
    INVALID_ADC_STOP_COUNT,
    PARKING,
)


class MotorController:
    def __init__(
        self,
        motors: Dict[str, object],
        spi: Optional[object] = None,
        sleep_fn: Callable[[float], None] = time.sleep,
        event_hook: Optional[Callable[[dict], None]] = None,
    ) -> None:
        self.motors = motors
        self.size = PWM_SIZE
        self.speed = DRIVE_SPEED
        self.full_at = FULL_AT_DEG
        self.max_steer = MAX_STEER
        self.direction_change_pause_sec = DIRECTION_CHANGE_PAUSE_SEC

        self.adc_right_full = ADC_RIGHT_FULL
        self.adc_right_half = ADC_RIGHT_HALF
        self.adc_center = ADC_CENTER
        self.adc_left_half = ADC_LEFT_HALF
        self.adc_left_full = ADC_LEFT_FULL
        # Compatibility with parking code.
        self.resistance_most_right = self.adc_right_full
        self.resistance_center = self.adc_center
        self.resistance_most_left = self.adc_left_full

        self.stop_band_adc = float(ADC_STOP_BAND)
        self.start_band_adc = float(ADC_START_BAND)
        self.steering_duty = float(STEERING_DUTY_PERCENT)
        self.pulse_model = STEERING_PULSE_MODEL
        self.prediction_safety_factor = float(STEERING_PREDICTION_SAFETY_FACTOR)
        self.reversal_settle_s = float(STEERING_REVERSAL_SETTLE_MS) / 1000.0
        self.minimum_command_interval_s = float(STEERING_MIN_COMMAND_INTERVAL_MS) / 1000.0
        self.invalid_adc_stop_count = int(INVALID_ADC_STOP_COUNT)

        self.manual_speed = 0.0
        self.current_drive_direction = "stop"
        self.steer_state = "straight"
        self.invalid_adc_count = 0
        self.last_valid_adc: Optional[int] = None
        self.last_pulse_direction: Optional[str] = None
        self.last_pulse_end = 0.0
        self.sleep = sleep_fn
        self.event_hook = event_hook

        if spi is not None:
            self.spi = spi
        else:
            if spidev is None:
                raise RuntimeError("spidev unavailable; run this file on the Ultra96 board")
            self.spi = spidev.SpiDev()
            self.spi.open(0, 0)
            self.spi.max_speed_hz = 20_000_000
            self.spi.mode = 0b00

    def _event(self, kind: str, **payload: object) -> None:
        if self.event_hook is not None:
            self.event_hook({"kind": kind, **payload})

    def duty_from_percent(self, percent: float) -> int:
        return int(self.size * max(0.0, min(100.0, float(percent))) / 100.0)

    def set_motor(self, name: str, percent: float = 0.0, valid: bool = False) -> None:
        motor = self.motors[name]
        motor.write(0x00, self.size)
        motor.write(0x04, self.duty_from_percent(percent))
        motor.write(0x08, 1 if valid else 0)

    def init_motors(self) -> None:
        self.stop_all()

    def stop_drive(self) -> None:
        for name in ("motor_0", "motor_1", "motor_2", "motor_3"):
            self.set_motor(name, 0, False)
        self.current_drive_direction = "stop"

    def steer_off(self) -> None:
        self.set_motor("motor_4", 0, False)
        self.set_motor("motor_5", 0, False)

    def stop_all(self) -> None:
        self.stop_drive()
        self.steer_off()
        self.steer_state = "straight"
        self.manual_speed = 0.0
        self.last_pulse_direction = None

    reset_motor_values = stop_all

    def _opposite_direction(self, new_direction: str) -> bool:
        return (
            self.current_drive_direction == "forward" and new_direction == "backward"
        ) or (
            self.current_drive_direction == "backward" and new_direction == "forward"
        )

    def prepare_direction_change(self, new_direction: str) -> None:
        if new_direction not in ("forward", "backward"):
            raise ValueError(new_direction)
        if self._opposite_direction(new_direction):
            self.stop_drive()
            self.sleep(self.direction_change_pause_sec)

    def set_drive(self, direction: str, speed: float) -> None:
        if direction == "stop" or speed == 0:
            self.stop_drive()
            return
        if direction not in ("forward", "backward"):
            raise ValueError(direction)
        self.prepare_direction_change(direction)
        duty = abs(float(speed))
        if direction == "forward":
            self.set_motor("motor_0", 0, False)
            self.set_motor("motor_3", 0, False)
            self.set_motor("motor_1", duty, True)
            self.set_motor("motor_2", duty, True)
        else:
            self.set_motor("motor_1", 0, False)
            self.set_motor("motor_2", 0, False)
            self.set_motor("motor_0", duty, True)
            self.set_motor("motor_3", duty, True)
        self.current_drive_direction = direction

    def forward(self, speed: float = DRIVE_SPEED) -> None:
        self.set_drive("forward", speed)

    def backward(self, speed: float = DRIVE_SPEED) -> None:
        self.set_drive("backward", speed)

    def steer_left(self, speed: Optional[float] = None) -> None:
        duty = self.steering_duty if speed is None else float(speed)
        self.set_motor("motor_4", 0, False)
        self.set_motor("motor_5", duty, True)

    def steer_right(self, speed: Optional[float] = None) -> None:
        duty = self.steering_duty if speed is None else float(speed)
        self.set_motor("motor_5", 0, False)
        self.set_motor("motor_4", duty, True)

    stay = steer_off

    def read_adc(self) -> int:
        response = self.spi.xfer2([0x00, 0x00])
        return ((response[0] & 0x0F) << 8) | response[1]

    @staticmethod
    def valid_adc_value(adc: Optional[int]) -> bool:
        return adc is not None and 0 < int(adc) < 4095

    @staticmethod
    def _lerp(x: float, x0: float, x1: float, y0: float, y1: float) -> float:
        if x1 == x0:
            return y0
        ratio = (x - x0) / (x1 - x0)
        return y0 + ratio * (y1 - y0)

    def adc_to_steer(self, adc: float) -> float:
        """Piecewise mapping using three measured anchors and two derived HALF targets: ADC -> steering units (-7..+7)."""
        x = float(adc)
        table = [
            (float(self.adc_right_full), 7.0),
            (float(self.adc_right_half), 3.5),
            (float(self.adc_center), 0.0),
            (float(self.adc_left_half), -3.5),
            (float(self.adc_left_full), -7.0),
        ]
        if not all(table[i][0] < table[i + 1][0] for i in range(4)):
            raise ValueError(f"invalid ADC point order: {table}")
        if x <= table[0][0]:
            return table[0][1]
        if x >= table[-1][0]:
            return table[-1][1]
        for (x0, y0), (x1, y1) in zip(table, table[1:]):
            if x <= x1:
                return self._lerp(x, x0, x1, y0, y1)
        return table[-1][1]

    def steer_to_adc(self, steer: float) -> float:
        """Inverse mapping using three measured anchors and two derived HALF targets: steering units (-7..+7) -> target ADC."""
        s = max(-7.0, min(7.0, float(steer)))
        table = [
            (-7.0, float(self.adc_left_full)),
            (-3.5, float(self.adc_left_half)),
            (0.0, float(self.adc_center)),
            (3.5, float(self.adc_right_half)),
            (7.0, float(self.adc_right_full)),
        ]
        for (s0, a0), (s1, a1) in zip(table, table[1:]):
            if s <= s1:
                return self._lerp(s, s0, s1, a0, a1)
        return table[-1][1]

    def map_angle_to_range(self, angle: float) -> float:
        deviation = 90.0 - abs(float(angle))
        steer = self.max_steer * deviation / self.full_at
        steer = max(0.0, min(self.max_steer, steer))
        return -steer if angle < 0 else steer

    def _adc_region(self, adc: float) -> str:
        right_boundary = (self.adc_right_half + self.adc_center) / 2.0
        left_boundary = (self.adc_center + self.adc_left_half) / 2.0
        if adc < right_boundary:
            return "right_region"
        if adc > left_boundary:
            return "left_region"
        return "center_region"

    def _next_steer_state_adc(self, error_adc: float) -> str:
        state = self.steer_state
        if abs(error_adc) <= self.stop_band_adc:
            return "straight"
        if state == "straight":
            if error_adc >= self.start_band_adc:
                return "left"  # increasing ADC
            if error_adc <= -self.start_band_adc:
                return "right"  # decreasing ADC
            return "straight"
        if state == "left" and error_adc <= -self.start_band_adc:
            return "right"
        if state == "right" and error_adc >= self.start_band_adc:
            return "left"
        return state

    def _model_entries(self, direction: str, region: str) -> list[dict]:
        by_direction = self.pulse_model.get(direction, {}) if isinstance(self.pulse_model, dict) else {}
        entries = by_direction.get(region) or by_direction.get("center_region") or []
        valid = []
        for item in entries:
            try:
                pulse_ms = float(item["pulse_ms"])
                movement = float(item.get("p90_abs_total_adc", abs(item["median_total_adc"])))
                if pulse_ms > 0 and movement > 0:
                    valid.append({**item, "pulse_ms": pulse_ms, "p90_abs_total_adc": movement})
            except Exception:
                continue
        return sorted(valid, key=lambda x: (x["p90_abs_total_adc"], x["pulse_ms"]))

    def select_pulse_ms(self, direction: str, current_adc: float, error_adc: float) -> Optional[float]:
        """Select the largest calibrated pulse predicted not to overshoot.

        p90 movement is used rather than the mean.  The additional safety factor
        intentionally under-commands near the target.
        """
        desired_move = max(0.0, abs(float(error_adc)) - self.stop_band_adc)
        allowed_move = desired_move * self.prediction_safety_factor
        entries = self._model_entries(direction, self._adc_region(current_adc))
        candidates = [e for e in entries if e["p90_abs_total_adc"] <= allowed_move]
        if not candidates:
            return None
        return float(max(candidates, key=lambda e: e["p90_abs_total_adc"])["pulse_ms"])

    def _pulse_direction(self, direction: str, pulse_ms: float) -> None:
        pulse_s = max(0.0, float(pulse_ms)) / 1000.0
        if pulse_s <= 0:
            return
        try:
            if direction == "left":
                self.steer_left(self.steering_duty)
            elif direction == "right":
                self.steer_right(self.steering_duty)
            else:
                raise ValueError(direction)
            self.sleep(pulse_s)
        finally:
            self.steer_off()
        self.last_pulse_direction = direction
        self.last_pulse_end = time.monotonic()
        self._event("steering_pulse", direction=direction, pulse_ms=pulse_ms, duty=self.steering_duty)

    def compute_steering_action(self, angle: float, adc: int) -> Tuple[str, float, float, float, float]:
        current_steer = self.adc_to_steer(adc)
        target_steer = self.map_angle_to_range(angle)
        target_adc = self.steer_to_adc(target_steer)
        error_adc = target_adc - float(adc)
        action = self._next_steer_state_adc(error_adc)
        return action, current_steer, target_steer, target_adc, error_adc

    def control_motors(self, angle: float, control_mode: int = 1) -> dict:
        del control_mode
        # Rear drive is kept continuous while the steering pulse is short.
        self.forward(self.speed)
        adc = self.read_adc()
        if not self.valid_adc_value(adc):
            self.invalid_adc_count += 1
            self.steer_off()
            if self.invalid_adc_count >= self.invalid_adc_stop_count:
                self.stop_all()
                raise RuntimeError(f"ADC invalid {self.invalid_adc_count} consecutive samples")
            return {"adc": adc, "action": "adc_invalid_hold"}

        self.invalid_adc_count = 0
        self.last_valid_adc = adc
        action, current_steer, target_steer, target_adc, error_adc = self.compute_steering_action(angle, adc)
        now = time.monotonic()
        pulse_ms: Optional[float] = None
        reason = ""

        if action == "straight":
            self.steer_off()
            self.steer_state = "straight"
            reason = "inside_deadband"
        else:
            if (
                self.last_pulse_direction is not None
                and action != self.last_pulse_direction
                and now - self.last_pulse_end < self.reversal_settle_s
            ):
                self.steer_off()
                self.steer_state = "straight"
                reason = "reversal_settle"
            elif now - self.last_pulse_end < self.minimum_command_interval_s:
                self.steer_off()
                reason = "minimum_interval"
            else:
                pulse_ms = self.select_pulse_ms(action, adc, error_adc)
                if pulse_ms is None:
                    self.steer_off()
                    self.steer_state = "straight"
                    reason = "no_safe_pulse"
                else:
                    self.steer_state = action
                    self._pulse_direction(action, pulse_ms)
                    reason = "pulse_applied"

        info = {
            "adc": adc,
            "current_steer": current_steer,
            "target_steer": target_steer,
            "target_adc": target_adc,
            "error_adc": error_adc,
            "action": action,
            "pulse_ms": pulse_ms,
            "reason": reason,
        }
        print(
            f"ADC={adc} targetADC={target_adc:.1f} errorADC={error_adc:+.1f} "
            f"steer={current_steer:+.2f}->{target_steer:+.2f} "
            f"action={action} pulse={pulse_ms} reason={reason}"
        )
        return info

    def handle_manual_control(self) -> None:
        import keyboard
        if keyboard.is_pressed("w"):
            self.manual_speed = min(self.manual_speed + 2, 100)
        if keyboard.is_pressed("s"):
            self.manual_speed = max(self.manual_speed - 2, -100)
        if keyboard.is_pressed("r"):
            self.manual_speed = 0

        if keyboard.is_pressed("a"):
            self.steer_left(100)
        elif keyboard.is_pressed("d"):
            self.steer_right(100)
        else:
            self.steer_off()

        if self.manual_speed > 0:
            self.forward(self.manual_speed)
        elif self.manual_speed < 0:
            self.backward(abs(self.manual_speed))
        else:
            self.stop_drive()

    def move_to_adc(
        self,
        target_adc: float,
        timeout: float = 5.0,
        cancel_check: Callable[[], bool] = lambda: False,
        settle_s: Optional[float] = None,
    ) -> bool:
        """Position steering to a target ADC using the same calibrated pulse model."""
        settle = self.reversal_settle_s if settle_s is None else float(settle_s)
        start = time.monotonic()
        confirms = 0
        last_direction: Optional[str] = None
        try:
            while time.monotonic() - start < timeout:
                if cancel_check():
                    return False
                adc = self.read_adc()
                if not self.valid_adc_value(adc):
                    self.steer_off()
                    self.sleep(0.02)
                    continue
                error = float(target_adc) - adc
                if abs(error) <= self.stop_band_adc:
                    self.steer_off()
                    confirms += 1
                    if confirms >= 3:
                        return True
                    self.sleep(0.02)
                    continue
                confirms = 0
                direction = "left" if error > 0 else "right"
                if last_direction and direction != last_direction:
                    self.steer_off()
                    self.sleep(settle)
                pulse_ms = self.select_pulse_ms(direction, adc, error)
                if pulse_ms is None:
                    # A calibrated minimum pulse may be too large near target; that is
                    # treated as within practical positioning resolution.
                    return abs(error) <= self.start_band_adc
                self._pulse_direction(direction, pulse_ms)
                last_direction = direction
                self.sleep(0.03)
            return False
        finally:
            self.steer_off()

    def center_wheels(
        self,
        timeout: float = 5.0,
        cancel_check: Callable[[], bool] = lambda: False,
    ) -> bool:
        return self.move_to_adc(
            self.adc_center,
            timeout=timeout,
            cancel_check=cancel_check,
            settle_s=PARKING["center_settle_sec"],
        )
