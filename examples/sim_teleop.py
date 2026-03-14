"""MuJoCo sim teleop: Joylo drives a simulated Franka in a 3D viewer.

While the viewer is running, type commands in the terminal:
  0-6   flip the sign for that joint
  s     save current signs to config.json
  q     quit
"""

import argparse
import json
import sys
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from franka_joylo import Joylo, JoyloSystem
from franka_joylo.sim import SimFrankaInterface

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.json"


def _input_loop(joylo: Joylo, config_path: str, stop_event: threading.Event):
    """Read terminal commands to flip joint signs and save config."""
    while not stop_event.is_set():
        try:
            cmd = input().strip()
        except EOFError:
            break

        if cmd in ("q", "quit"):
            stop_event.set()
            break
        elif cmd == "s":
            _save_signs(config_path, joylo.joint_signs)
            print(f"  Saved signs to {config_path}")
        elif cmd in [str(i) for i in range(7)]:
            idx = int(cmd)
            joylo.flip_joint_sign(idx)
            signs = joylo.joint_signs
            label = "+" if signs[idx] > 0 else "-"
            print(f"  Joint {idx} sign -> {label}1   (all: {_fmt_signs(signs)})")
        else:
            print("  Commands: 0-6 (flip joint), s (save), q (quit)")


def _save_signs(config_path: str, signs: np.ndarray):
    """Update just the joint_signs field in the config file."""
    with open(config_path) as f:
        config = json.load(f)
    config["joint_signs"] = signs.tolist()
    with open(config_path, "w") as f:
        json.dump(config, f, indent=2)


def _fmt_signs(signs: np.ndarray) -> str:
    return "[" + ", ".join(f"{'+' if s > 0 else '-'}1" for s in signs) + "]"


def main():
    parser = argparse.ArgumentParser(description="Sim teleop: Joylo drives MuJoCo Franka")
    parser.add_argument("--config", type=str, default=str(CONFIG_PATH),
                        help="Path to config.json from calibration")
    parser.add_argument("--rate", type=float, default=100.0,
                        help="Control loop rate in Hz")
    parser.add_argument("--alpha", type=float, default=0.95,
                        help="EMA smoothing factor (1.0 = no smoothing)")
    args = parser.parse_args()

    # Load calibration config
    with open(args.config) as f:
        config = json.load(f)

    print(f"Loaded config from {args.config}")
    print(f"  5V port:  {config['port_5v']}")
    print(f"  12V port: {config['port_12v']}")

    # Create sim Franka
    sim_franka = SimFrankaInterface.create()
    model, data = sim_franka.model, sim_franka.data

    # Set initial control to home pose so the sim doesn't collapse
    data.ctrl[:] = data.qpos[:7]

    # Create Joylo
    joylo = Joylo(
        port_5v=config["port_5v"],
        port_12v=config["port_12v"],
        joint_signs=np.array(config["joint_signs"]),
        joint_offsets_rad=np.array(config["joint_offsets_rad"]),
        gravity_comp_currents=np.array(config.get("gravity_comp_currents", [0]*7), dtype=int),
    )

    # Create system
    system = JoyloSystem(sim_franka, joylo,
                         control_rate_hz=args.rate,
                         smoothing_alpha=args.alpha)

    print()
    print("Starting teleop (Joylo leads, sim Franka follows)...")
    print(f"Current signs: {_fmt_signs(joylo.joint_signs)}")
    print()
    print("Move the Joylo and watch the sim. If a joint moves the wrong way, flip it:")
    print("  0-6   flip that joint's sign")
    print("  s     save signs to config.json")
    print("  q     quit")
    print()

    system.start_teleop()

    stop_event = threading.Event()
    input_thread = threading.Thread(
        target=_input_loop, args=(joylo, args.config, stop_event), daemon=True,
    )
    input_thread.start()

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and not stop_event.is_set():
                step_start = time.time()
                mujoco.mj_step(model, data)
                viewer.sync()
                elapsed = time.time() - step_start
                sleep_time = model.opt.timestep - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        system.close()
        print("Done.")


if __name__ == "__main__":
    main()
