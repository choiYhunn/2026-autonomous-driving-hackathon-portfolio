from __future__ import annotations

import argparse
import sys
from pathlib import Path

from pynq import MMIO
from pynq_dpu import DpuOverlay

from AutoLab_lib import init
from config import (
    DRIVE_MOTOR_ADDRESSES,
    ADDRESS_RANGE,
    DRIVE_DPU_BIT,
    XMODEL,
    print_active_config,
)
from driving_system_controller import DrivingSystemController


def map_mmio(addresses):
    return {name: MMIO(address, ADDRESS_RANGE) for name, address in addresses.items()}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("autonomous", "manual"))
    args = parser.parse_args()
    mode = 1 if args.mode == "autonomous" else 2

    init()
    print_active_config()
    print(f"[DRIVE] bit: {DRIVE_DPU_BIT}")
    print(f"[DRIVE] model: {XMODEL}")
    if not Path(DRIVE_DPU_BIT).exists():
        raise FileNotFoundError(DRIVE_DPU_BIT)
    if not Path(XMODEL).exists():
        raise FileNotFoundError(XMODEL)

    # Load the verified normal DPU before creating 0x00A0... MMIO objects.
    overlay = DpuOverlay(DRIVE_DPU_BIT)
    overlay.load_model(XMODEL)
    motors = map_mmio(DRIVE_MOTOR_ADDRESSES)

    Path("result").mkdir(exist_ok=True)
    controller = DrivingSystemController(
        overlay,
        overlay.runner,
        motors,
        ultras=None,
        allowed_modes=(1, 2),
    )
    reason = controller.run(
        camera_index=0,
        save_path=f"result/{args.mode}.avi",
        initial_mode=mode,
        return_on_q=True,
    )
    return 10 if reason == "exit" else 0


if __name__ == "__main__":
    sys.exit(main())
