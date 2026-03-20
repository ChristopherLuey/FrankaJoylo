"""Real robot teleop: Joylo controls a Franka via the deoxys driver.

Usage:
    python examples/real_teleop.py [--config config.json] [--mode takeover|teleop|tracking]

Requires:
    - deoxys running on the NUC (auto_arm.sh config/charmander.yml)
    - Joylo controller connected via USB
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np

from franka_joylo import Joylo, JoyloSystem
from franka_joylo.deoxys_franka import DeoxysFrankaInterface

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.json"


def main():
    parser = argparse.ArgumentParser(description="Real robot teleop via deoxys")
    parser.add_argument("--config", type=str, default=str(CONFIG_PATH),
                        help="Path to config.json")
    parser.add_argument("--mode", choices=["takeover", "teleop", "tracking"],
                        default="takeover",
                        help="Operating mode (default: takeover)")
    parser.add_argument("--rate", type=float, default=100.0,
                        help="Control loop rate in Hz")
    parser.add_argument("--alpha", type=float, default=0.95,
                        help="EMA smoothing factor for teleop")
    args = parser.parse_args()

    # Load Joylo config
    with open(args.config) as f:
        config = json.load(f)

    print(f"Loaded config from {args.config}")
    print(f"  5V port:  {config['port_5v']}")
    print(f"  12V port: {config['port_12v']}")

    # Create deoxys Franka interface
    print("Connecting to Franka via deoxys...")
    franka = DeoxysFrankaInterface(control_freq=int(args.rate))

    # Wait for robot state
    print("Waiting for robot state...", end="", flush=True)
    while franka.robot_interface.state_buffer_size == 0:
        time.sleep(0.1)
    print(" OK")
    print(f"  Joint positions: {np.array2string(franka.read_joint_positions(), precision=3, separator=', ')}")

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
