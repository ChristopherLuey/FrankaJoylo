"""MuJoCo sim teleop: Joylo drives a simulated Franka in a 3D viewer."""

import argparse
import json
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from franka_joylo import Joylo, JoyloSystem
from franka_joylo.sim import SimFrankaInterface

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.json"


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

    print("Starting teleop (Joylo leads, sim Franka follows)...")
    print("Close the viewer window to stop.")
    system.start_teleop()

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
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
        system.close()
        print("Done.")


if __name__ == "__main__":
    main()
