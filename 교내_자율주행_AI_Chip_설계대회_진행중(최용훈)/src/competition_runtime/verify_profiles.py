"""Load one hardware profile without moving any motor."""
from __future__ import annotations
import argparse
from pathlib import Path
from pynq import MMIO, Overlay
from pynq_dpu import DpuOverlay
from config import *

def map_mmio(addresses):
    return {name: MMIO(address, ADDRESS_RANGE) for name, address in addresses.items()}

parser = argparse.ArgumentParser()
parser.add_argument("profile", choices=("drive", "parking"))
args = parser.parse_args()

if args.profile == "drive":
    print("bit:", DRIVE_DPU_BIT)
    overlay = DpuOverlay(DRIVE_DPU_BIT)
    overlay.load_model(XMODEL)
    mmio = map_mmio(DRIVE_MOTOR_ADDRESSES)
    print("PASS: normal DPU/model + 0x00A0 motor map loaded; no motor command sent")
else:
    print("bit:", PARKING_BIT)
    overlay = Overlay(PARKING_BIT)
    motors = map_mmio(PARKING_MOTOR_ADDRESSES)
    ultras = map_mmio(PARKING_ULTRASONIC_ADDRESSES)
    values = {name: int(dev.read(0x00)) for name, dev in ultras.items()}
    print("ultrasonic raw:", values)
    print("PASS: custom parking bit + 0x800 motor/ultrasonic map loaded; no motor command sent")
