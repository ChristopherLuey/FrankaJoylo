"""Interactive calibration script for the Joylo.

Workflow:
  1. Scans both USB ports and verifies motor connectivity
  2. Reads raw positions from all 7 motors
  3. Lets you determine joint_signs by moving each joint
  4. Prompts for joint_offsets_rad (Franka's current joint angles)
  5. Saves config.json and prints a ready-to-copy config dict
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


def determine_joint_signs(driver_5v: DxlDriver, driver_12v: DxlDriver) -> np.ndarray:
    """Interactively determine joint signs by asking user to move each joint."""
    all_ids = sorted(CONTROLLER_1_MOTOR_IDS + CONTROLLER_2_MOTOR_IDS)
    signs = np.ones(7)

    print("\n--- Joint Sign Calibration ---")
    print("For each joint, move it in the POSITIVE direction on the Franka")
    print("(i.e., the direction that increases the Franka joint angle).")
    print()

    for i, mid in enumerate(all_ids):
        input(f"  Joint {i} (motor {mid}): Press Enter, then move in Franka-positive direction...")

        # Read initial position
        pos_before = {**driver_5v.read_positions(), **driver_12v.read_positions()}
        raw_before = pos_before[mid]

        input(f"  Joint {i} (motor {mid}): Press Enter when done moving.")

        pos_after = {**driver_5v.read_positions(), **driver_12v.read_positions()}
        raw_after = pos_after[mid]

        delta = raw_after - raw_before
        if abs(delta) < 10:
            print(f"    WARNING: Very small movement detected ({delta} ticks). Try again or default to +1.")
            signs[i] = 1.0
        else:
            signs[i] = 1.0 if delta > 0 else -1.0
            direction = "+" if signs[i] > 0 else "-"
            print(f"    Detected delta={delta} ticks -> sign={direction}1")

    return signs


def main():
    parser = argparse.ArgumentParser(description="Joylo calibration utility")
    parser.add_argument("--port-5v", default="/dev/ttyUSB0", help="USB port for 5V controller (XL330)")
    parser.add_argument("--port-12v", default="/dev/ttyUSB1", help="USB port for 12V controller (XL430)")
    parser.add_argument("--baudrate", type=int, default=57600)
    parser.add_argument("--skip-signs", action="store_true", help="Skip joint sign calibration (assume all +1)")
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

    all_ids = sorted(CONTROLLER_1_MOTOR_IDS + CONTROLLER_2_MOTOR_IDS)
    merged = {**pos_5v, **pos_12v}
    print(f"\n  All 7 motors found!")

    # Step 2: Joint signs
    if args.skip_signs:
        signs = np.ones(7)
        print("\n=== Step 2: Joint signs (skipped, all +1) ===")
    else:
        print("\n=== Step 2: Joint sign calibration ===")
        driver_5v = DxlDriver(args.port_5v, CONTROLLER_1_MOTOR_IDS, args.baudrate)
        driver_12v = DxlDriver(args.port_12v, CONTROLLER_2_MOTOR_IDS, args.baudrate)
        signs = determine_joint_signs(driver_5v, driver_12v)
        driver_5v.close()
        driver_12v.close()

    # Step 3: Joint offsets
    print("\n=== Step 3: Joint offsets ===")
    print("Place the Joylo in its startup pose and read the Franka's current joint angles.")
    print("Enter the 7 Franka joint angles in radians, separated by spaces.")
    print("(If the Joylo startup pose matches Franka home [0,0,0,0,0,0,0], just press Enter)")
    print()
    raw_input = input("  Franka joint angles (rad): ").strip()
    if raw_input:
        offsets = np.array([float(x) for x in raw_input.split()])
        assert len(offsets) == 7, "Expected 7 values"
    else:
        offsets = np.zeros(7)

    # Step 4: Save config
    config = {
        "port_5v": args.port_5v,
        "port_12v": args.port_12v,
        "baudrate": args.baudrate,
        "joint_signs": signs.tolist(),
        "joint_offsets_rad": offsets.tolist(),
        "gravity_comp_currents": [0, 0, 0, 0, 0, 0, 0],
    }

    with open(config_path, "w") as f:
        json.dump(config, f, indent=2)
    print(f"\n=== Config saved to {config_path} ===")

    print("\nExample usage:\n")
    print(f"""
import json, numpy as np
from franka_joylo import Joylo

with open("{config_path}") as f:
    cfg = json.load(f)

joylo = Joylo(
    port_5v=cfg["port_5v"],
    port_12v=cfg["port_12v"],
    joint_signs=np.array(cfg["joint_signs"]),
    joint_offsets_rad=np.array(cfg["joint_offsets_rad"]),
)
""")


if __name__ == "__main__":
    main()
