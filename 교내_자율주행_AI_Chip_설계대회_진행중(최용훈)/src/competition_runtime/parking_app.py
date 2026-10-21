from __future__ import annotations

import sys
import time
from pathlib import Path

import keyboard
from pynq import MMIO, Overlay

from AutoLab_lib import init
from config import (
    PARKING_MOTOR_ADDRESSES,
    PARKING_ULTRASONIC_ADDRESSES,
    ADDRESS_RANGE,
    PARKING_BIT,
    print_active_config,
)
from motor_controller import MotorController
from parking_mission import ParkingMissionController


def map_mmio(addresses):
    return {name: MMIO(address, ADDRESS_RANGE) for name, address in addresses.items()}


def main() -> int:
    init()
    print_active_config()
    print(f"[PARK] bit: {PARKING_BIT}")
    if not Path(PARKING_BIT).exists():
        raise FileNotFoundError(PARKING_BIT)

    # Parking does not call DpuOverlay/load_model. The custom bit is used only
    # for its 0x800... motor and ultrasonic IP blocks.
    overlay = Overlay(PARKING_BIT)
    motors = map_mmio(PARKING_MOTOR_ADDRESSES)
    ultras = map_mmio(PARKING_ULTRASONIC_ADDRESSES)
    motor = MotorController(motors)
    mission = ParkingMissionController(motor, ultras)
    motor.stop_all()

    print("\n===== PARKING MODE =====")
    print("Space: start/stop parking mission")
    print("Q: stop and return to main menu")
    print("X or Esc: final exit")
    key_state = {}

    def pressed_once(key: str) -> bool:
        now = bool(keyboard.is_pressed(key))
        before = key_state.get(key, False)
        key_state[key] = now
        return now and not before

    final_exit = False
    try:
        while True:
            if pressed_once("space"):
                if mission.is_started:
                    mission.stop()
                else:
                    mission.start()
            if pressed_once("q"):
                mission.stop()
                break
            if pressed_once("x") or pressed_once("esc"):
                mission.stop()
                final_exit = True
                break
            time.sleep(0.02)
    except KeyboardInterrupt:
        mission.stop()
    finally:
        mission.stop()
        motor.stop_all()
        del overlay
        print("parking motors stopped and resources released")
    return 10 if final_exit else 0


if __name__ == "__main__":
    sys.exit(main())
