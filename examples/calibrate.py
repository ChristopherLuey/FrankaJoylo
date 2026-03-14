"""Interactive calibration script for the Joylo.

Workflow:
  1. Scans both USB ports and verifies motor connectivity
  2. Prompts for joint_offsets_rad (Franka's current joint angles)
  3. Saves config.json (joint signs default to +1 — tune them in the sim)
"""

import argparse
import json
from pathlib import Path

import numpy as np

from franka_joylo.constants import (
    CONTROLLER_1_MOTOR_IDS,
    CONTROLLER_2_MOTOR_IDS,
    DXL_TO_RAD,
)
from franka_joylo.dxl_driver import DxlDriver


def scan_motors(port: str, motor_ids: list[int], baudrate: int) -> dict[int, int]:
    """Try to connect and read positions. Returns {motor_id: raw_position}."""
    try:
        driver = DxlDriver(port, motor_ids, baudrate)
        positions = driver.read_positions()
        driver.close()
        return positions
    except RuntimeError as e:
        print(f"  ERROR on {port}: {e}")
        return {}


def main():
    parser = argparse.ArgumentParser(description="Joylo calibration utility")
    parser.add_argument("--port-5v", default="/dev/ttyUSB0", help="USB port for 5V controller (XL330)")
    parser.add_argument("--port-12v", default="/dev/ttyUSB1", help="USB port for 12V controller (XL430)")
    parser.add_argument("--baudrate", type=int, default=57600)
    parser.add_argument("--output", type=str, default=None,
                        help="Path to save config JSON (default: <project_root>/config.json)")
    args = parser.parse_args()

    config_path = args.output or str(Path(__file__).resolve().parent.parent / "config.json")

    # Step 1: Scan motors
    print("=== Step 1: Scanning motors ===")
    print(f"  5V controller ({args.port_5v}): motors {CONTROLLER_1_MOTOR_IDS}")
    pos_5v = scan_motors(args.port_5v, CONTROLLER_1_MOTOR_IDS, args.baudrate)
    if pos_5v:
        for mid, raw in sorted(pos_5v.items()):
            print(f"    Motor {mid}: raw={raw}  ({raw * DXL_TO_RAD:.3f} rad from center)")
    else:
        print("    FAILED — check port and connections")
        return

    print(f"  12V controller ({args.port_12v}): motors {CONTROLLER_2_MOTOR_IDS}")
    pos_12v = scan_motors(args.port_12v, CONTROLLER_2_MOTOR_IDS, args.baudrate)
    if pos_12v:
        for mid, raw in sorted(pos_12v.items()):
            print(f"    Motor {mid}: raw={raw}  ({raw * DXL_TO_RAD:.3f} rad from center)")
    else:
        print("    FAILED — check port and connections")
        return

    print(f"\n  All 7 motors found!")

    # Step 2: Joint offsets
    print("\n=== Step 2: Joint offsets ===")
    print("Enter the Franka's current joint angles (radians) that match")
    print("the Joylo's current pose. 7 values separated by spaces.")
    print("(Press Enter to use all zeros)")
    print()
    raw_input = input("  Franka joint angles (rad): ").strip()
    if raw_input:
        offsets = np.array([float(x) for x in raw_input.split()])
        assert len(offsets) == 7, "Expected 7 values"
    else:
        offsets = np.zeros(7)

    # Step 3: Save config
    config = {
        "port_5v": args.port_5v,
        "port_12v": args.port_12v,
        "baudrate": args.baudrate,
        "joint_signs": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        "joint_offsets_rad": offsets.tolist(),
        "gravity_comp_currents": [0, 0, 0, 0, 0, 0, 0],
    }

    with open(config_path, "w") as f:
        json.dump(config, f, indent=2)

    print(f"\nConfig saved to {config_path}")
    print("Joint signs are all +1. Tune them in the sim viewer (press 0-6 to flip).")


if __name__ == "__main__":
    main()
