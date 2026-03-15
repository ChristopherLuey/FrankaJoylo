"""Calibration script for the Joylo.

Workflow:
  1. Pings each motor ID on both USB ports to verify connectivity
  2. Saves config.json with default signs (+1) and offsets (0)
  3. Signs and offsets are tuned live in the sim viewer (sim_teleop.py)
"""

import argparse
import json
import os
from pathlib import Path

from dynamixel_sdk import PacketHandler, PortHandler

from franka_joylo.constants import (
    MOTOR_IDS_5V,
    MOTOR_IDS_12V,
    DXL_TO_RAD,
)
from franka_joylo.dxl_driver import DxlDriver

PROTOCOL_VERSION = 2.0
SCAN_RANGE = range(10)


def resolve_port(port: str) -> str:
    """Resolve symlinks and print what the port actually points to."""
    real = os.path.realpath(port)
    if real != port:
        print(f"    {port} -> {real}")
    if not os.path.exists(real):
        print(f"    WARNING: {real} does not exist!")
    return real


def ping_scan(port: str, baudrate: int) -> list[int]:
    """Ping motor IDs 0-9 on a port and return the list of IDs that respond."""
    resolve_port(port)

    port_handler = PortHandler(port)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    if not port_handler.openPort():
        print(f"    ERROR: Could not open {port}")
        return []
    if not port_handler.setBaudRate(baudrate):
        port_handler.closePort()
        print(f"    ERROR: Could not set baudrate on {port}")
        return []

    found = []
    for mid in SCAN_RANGE:
        _, result, _ = packet_handler.ping(port_handler, mid)
        if result == 0:
            found.append(mid)

    port_handler.closePort()
    return found


def scan_motors(port: str, motor_ids: list[int], baudrate: int) -> dict[int, int]:
    """Try to connect and read positions. Returns {motor_id: raw_position}."""
    try:
        driver = DxlDriver(port, motor_ids, baudrate)
        positions = driver.read_positions()
    except RuntimeError as e:
        print(f"    ERROR on {port}: {e}")
        return {}
    finally:
        try:
            driver.close()
        except Exception:
            pass
    return positions


def main():
    parser = argparse.ArgumentParser(description="Joylo calibration utility")
    parser.add_argument("--port-5v", default="/dev/franka_small", help="USB port for 5V controller (XL330)")
    parser.add_argument("--port-12v", default="/dev/franka_large", help="USB port for 12V controller (XL430)")
    parser.add_argument("--baudrate", type=int, default=1000000)
    parser.add_argument("--output", type=str, default=None,
                        help="Path to save config JSON (default: <project_root>/config.json)")
    parser.add_argument("--scan-only", action="store_true",
                        help="Just ping-scan both ports and exit (for debugging)")
    args = parser.parse_args()

    config_path = args.output or str(Path(__file__).resolve().parent.parent / "config.json")

    # Step 1: Ping scan to show what's actually on each bus
    print("=== Step 1: Scanning motors ===")

    print(f"  Pinging IDs {list(SCAN_RANGE)} on {args.port_5v}...")
    found_5v = ping_scan(args.port_5v, args.baudrate)
    print(f"    Responded: {found_5v}  (expected {MOTOR_IDS_5V})")

    print(f"  Pinging IDs {list(SCAN_RANGE)} on {args.port_12v}...")
    found_12v = ping_scan(args.port_12v, args.baudrate)
    print(f"    Responded: {found_12v}  (expected {MOTOR_IDS_12V})")

    if args.scan_only:
        return

    # Check that expected IDs are present
    missing_5v = set(MOTOR_IDS_5V) - set(found_5v)
    missing_12v = set(MOTOR_IDS_12V) - set(found_12v)

    if missing_5v or missing_12v:
        if missing_5v:
            print(f"\n  ERROR: 5V port missing motor IDs {sorted(missing_5v)}")
        if missing_12v:
            print(f"\n  ERROR: 12V port missing motor IDs {sorted(missing_12v)}")
        if set(MOTOR_IDS_5V).issubset(set(found_12v)) and set(MOTOR_IDS_12V).issubset(set(found_5v)):
            print("\n  HINT: Ports look swapped! Try swapping --port-5v and --port-12v,")
            print("  or re-run setup.sh and unplug the correct controller.")
        return

    # Read positions
    print(f"\n  Reading positions on 5V ({args.port_5v})...")
    pos_5v = scan_motors(args.port_5v, MOTOR_IDS_5V, args.baudrate)
    if pos_5v:
        for mid, raw in sorted(pos_5v.items()):
            print(f"    Motor {mid}: raw={raw}  ({raw * DXL_TO_RAD:.3f} rad)")
    else:
        print("    FAILED — check connections")
        return

    print(f"  Reading positions on 12V ({args.port_12v})...")
    pos_12v = scan_motors(args.port_12v, MOTOR_IDS_12V, args.baudrate)
    if pos_12v:
        for mid, raw in sorted(pos_12v.items()):
            print(f"    Motor {mid}: raw={raw}  ({raw * DXL_TO_RAD:.3f} rad)")
    else:
        print("    FAILED — check connections")
        return

    print(f"\n  All 7 motors found!")

    # Step 2: Save config
    config = {
        "port_5v": args.port_5v,
        "port_12v": args.port_12v,
        "baudrate": args.baudrate,
        "joint_signs": [1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        "joint_offsets_rad": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "gravity_comp_currents": [0, 0, 0, 0, 0, 0, 0],
    }

    with open(config_path, "w") as f:
        json.dump(config, f, indent=2)

    print(f"\nConfig saved to {config_path}")
    print("Tune joint signs and offsets in the sim viewer (sim_teleop.py).")


if __name__ == "__main__":
    main()
