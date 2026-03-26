"""Real robot teleop: Joylo controls a Franka via the deoxys driver.

Usage:
    python examples/real_teleop.py [--config config.json] [--mode takeover|teleop|tracking|home]

Requires:
    - deoxys running on the NUC (auto_arm.sh config/charmander.yml)
    - Joylo controller connected via USB (not needed for home mode)
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np

from franka_joylo import Joylo, JoyloSystem
from franka_joylo.constants import FRANKA_HOME
from franka_joylo.deoxys_franka import DeoxysFrankaInterface

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.json"

MAX_JOINT_VEL = 0.15  # rad/s — slow and safe max velocity per joint
MIN_DURATION = 3.0    # seconds — never faster than this


def move_to(franka: DeoxysFrankaInterface, target: np.ndarray, rate_hz: float):
    """Smoothly interpolate the Franka from its current pose to *target*."""
    start = franka.read_joint_positions()
    dist = np.max(np.abs(target - start))
    duration = max(dist / MAX_JOINT_VEL, MIN_DURATION)
    steps = int(duration * rate_hz)
    dt = 1.0 / rate_hz

    print(f"  Moving {dist:.2f} rad max displacement over {duration:.1f}s ({steps} steps)")

    for i in range(1, steps + 1):
        t0 = time.monotonic()
        alpha = i / steps
        smooth = 0.5 * (1.0 - np.cos(np.pi * alpha))
        waypoint = start + smooth * (target - start)
        franka.send_joint_positions(waypoint)
        elapsed = time.monotonic() - t0
        if elapsed < dt:
            time.sleep(dt - elapsed)

    franka.send_joint_positions(target)


def main():
    parser = argparse.ArgumentParser(description="Real robot teleop via deoxys")
    parser.add_argument("--config", type=str, default=str(CONFIG_PATH),
                        help="Path to config.json")
    parser.add_argument("--mode", choices=["takeover", "teleop", "tracking", "home"],
                        default="takeover",
                        help="Operating mode (default: takeover)")
    parser.add_argument("--rate", type=float, default=100.0,
                        help="Control loop rate in Hz")
    parser.add_argument("--alpha", type=float, default=0.95,
                        help="EMA smoothing factor for teleop")
    args = parser.parse_args()

    # Create deoxys Franka interface
    print("Connecting to Franka via deoxys...")
    franka = DeoxysFrankaInterface(control_freq=int(args.rate))

    # Wait for robot state
    print("Waiting for robot state...", end="", flush=True)
    while franka.robot_interface.state_buffer_size == 0:
        time.sleep(0.1)
    print(" OK")
    print(f"  Joint positions: {np.array2string(franka.read_joint_positions(), precision=3, separator=', ')}")

    if args.mode == "home":
        print("\nMoving to home pose...")
        try:
            move_to(franka, FRANKA_HOME, args.rate)
            print("  Home.")
        except KeyboardInterrupt:
            print("\n  Interrupted.")
        finally:
            franka.close()
            print("Done.")
        return

    # Load Joylo config (only needed for teleop/tracking/takeover)
    with open(args.config) as f:
        config = json.load(f)

    print(f"Loaded config from {args.config}")
    print(f"  5V port:  {config['port_5v']}")
    print(f"  12V port: {config['port_12v']}")

    # Create Joylo
    joylo = Joylo(
        port_5v=config["port_5v"],
        port_12v=config["port_12v"],
        joint_signs=np.array(config["joint_signs"]),
        joint_offsets_rad=np.array(config["joint_offsets_rad"]),
        gravity_comp_currents=np.array(config.get("gravity_comp_currents", [0] * 7), dtype=int),
    )

    system = JoyloSystem(franka, joylo,
                         control_rate_hz=args.rate,
                         smoothing_alpha=args.alpha)

    try:
        if args.mode == "tracking":
            print("\nTracking mode: Franka leads, Joylo follows.")
            print("Press Enter to stop.\n")
            system.start_tracking()
            input()

        elif args.mode == "teleop":
            print("\nTeleop mode: Joylo leads, Franka follows.")
            print("Press Enter to stop.\n")
            system.start_teleop()
            input()

        elif args.mode == "takeover":
            print("\nTakeover mode: starting in tracking (Franka leads).")
            print("Press Enter to switch to teleop...\n")

            triggered = False

            def trigger_fn() -> bool:
                return triggered

            system.start_tracking(trigger_fn=trigger_fn)
            input()
            triggered = True
            # Wait for the tracking loop to notice the trigger and transition
            time.sleep(0.2)

            print("Switched to teleop. Joylo now leads.")
            print("Press Enter to stop.\n")
            input()

    except KeyboardInterrupt:
        pass
    finally:
        system.close()
        franka.close()
        print("Done.")


if __name__ == "__main__":
    main()
