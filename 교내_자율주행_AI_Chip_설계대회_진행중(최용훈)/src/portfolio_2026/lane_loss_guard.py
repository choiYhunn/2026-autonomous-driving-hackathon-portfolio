"""Time-based lane-loss fallback separated from perception and control code."""

from dataclasses import dataclass


@dataclass(frozen=True)
class GuardDecision:
    use_straight_fallback: bool
    reset_control_history: bool
    lost_duration_s: float


class LaneLossGuard:
    def __init__(self, timeout_s: float = 3.0) -> None:
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        self.timeout_s = float(timeout_s)
        self._lost_since: float | None = None
        self._fallback_active = False

    def update(self, detected: bool, now_s: float) -> GuardDecision:
        if detected:
            reset = self._fallback_active
            self._lost_since = None
            self._fallback_active = False
            return GuardDecision(False, reset, 0.0)

        if self._lost_since is None:
            self._lost_since = now_s
        duration = max(0.0, now_s - self._lost_since)
        just_activated = duration >= self.timeout_s and not self._fallback_active
        if duration >= self.timeout_s:
            self._fallback_active = True
        return GuardDecision(self._fallback_active, just_activated, duration)

