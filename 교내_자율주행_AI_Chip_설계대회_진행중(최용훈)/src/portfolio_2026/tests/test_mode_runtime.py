import unittest

from portfolio_2026.mode_runtime import Mode, ModeCoordinator


class FakeRuntime:
    def __init__(self):
        self.events = []

    def start(self): self.events.append("start")
    def stop(self): self.events.append("stop")
    def close(self): self.events.append("close")


class ModeRuntimeTest(unittest.TestCase):
    def test_switch_closes_previous_runtime(self):
        drive, park = FakeRuntime(), FakeRuntime()
        coordinator = ModeCoordinator({Mode.DRIVE: lambda: drive, Mode.PARK: lambda: park})
        coordinator.select(Mode.DRIVE)
        coordinator.start()
        coordinator.select(Mode.PARK)
        self.assertEqual(drive.events, ["start", "stop", "close"])
        self.assertEqual(park.events, [])


if __name__ == "__main__":
    unittest.main()

