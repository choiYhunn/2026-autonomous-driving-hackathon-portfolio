"""TEAM08 dual-overlay launcher.

Driving/manual and parking run in separate child processes. This is intentional:
each child loads its own bitstream first and maps only the addresses that exist
in that bitstream.
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent


def _run(script: str, *args: str) -> int:
    command = [sys.executable, str(BASE_DIR / script), *args]
    print("[LAUNCH]", " ".join(command))
    completed = subprocess.run(command, cwd=str(BASE_DIR))
    return int(completed.returncode)


def main() -> None:
    while True:
        print("\n===== TEAM08 BITSTREAM MENU =====")
        print("1: autonomous driving  [normal ../dpu/dpu.bit + 0x00A0...]")
        print("2: manual driving      [normal ../dpu/dpu.bit + 0x00A0...]")
        print("3: parking mission     [custom dpu_cyh_2/dpu.bit + 0x800...]")
        print("x: final exit")
        choice = input("select> ").strip().lower()

        if choice == "1":
            code = _run("drive_app.py", "autonomous")
        elif choice == "2":
            code = _run("drive_app.py", "manual")
        elif choice == "3":
            code = _run("parking_app.py")
        elif choice in ("x", "q", "exit"):
            print("final exit")
            return
        else:
            print("invalid selection")
            continue

        # Child code 10 means X/Esc was requested, not merely Q-to-menu.
        if code == 10:
            print("final exit requested")
            return
        if code != 0:
            print(f"[WARN] child exited with code {code}")


if __name__ == "__main__":
    main()
