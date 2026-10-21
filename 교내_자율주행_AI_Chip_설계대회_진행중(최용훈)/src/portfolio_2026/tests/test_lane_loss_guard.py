import unittest

from portfolio_2026.lane_loss_guard import LaneLossGuard


class LaneLossGuardTest(unittest.TestCase):
    def test_fallback_after_timeout_and_reset_on_recovery(self):
        guard = LaneLossGuard(timeout_s=3.0)
        self.assertFalse(guard.update(False, 10.0).use_straight_fallback)
        self.assertFalse(guard.update(False, 12.9).use_straight_fallback)
        activated = guard.update(False, 13.0)
        self.assertTrue(activated.use_straight_fallback)
        self.assertTrue(activated.reset_control_history)
        recovered = guard.update(True, 13.1)
        self.assertFalse(recovered.use_straight_fallback)
        self.assertTrue(recovered.reset_control_history)


if __name__ == "__main__":
    unittest.main()

