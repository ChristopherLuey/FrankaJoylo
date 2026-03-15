"""MuJoCo sim teleop: Joylo drives a simulated Franka in a 3D viewer.

Interactive tuning — type commands in the terminal while watching the sim:

  0-6       select active joint
  f         flip sign of active joint
  z         snap all offsets (freeze sim at home, match Joylo, press Enter)
  + / -     nudge offset of active joint by +/- 0.1 rad
  ] / [     nudge offset of active joint by +/- 0.01 rad (fine)
  r         reset offset of active joint to 0
  s         save config to disk
  q         quit
"""

import argparse
import json
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from franka_joylo import Joylo, JoyloSystem
from franka_joylo.sim import SimFrankaInterface

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.json"

COARSE_STEP = 0.1   # radians
FINE_STEP = 0.01    # radians


def _print_status(joylo: Joylo, active: int):
    signs = joylo.joint_signs
    offsets = joylo.joint_offsets_rad
    parts = []
    for i in range(7):
        s = "+" if signs[i] > 0 else "-"
        marker = "*" if i == active else " "
        parts.append(f"{marker}J{i}: {s}1  {offsets[i]:+.3f}")
    print("  " + "  |  ".join(parts))


def _save_config(config_path: str, joylo: Joylo):
    with open(config_path) as f:
        config = json.load(f)
    config["joint_signs"] = joylo.joint_signs.tolist()
    config["joint_offsets_rad"] = joylo.joint_offsets_rad.tolist()
    with open(config_path, "w") as f:
        json.dump(config, f, indent=2)


def _input_loop(joylo: Joylo, system: JoyloSystem, sim_franka: SimFrankaInterface,
                home_qpos: np.ndarray, config_path: str, stop_event: threading.Event):
    active = 0
    _print_status(joylo, active)

    while not stop_event.is_set():
        try:
            cmd = input("> ").strip()
        except EOFError:
            break

        if cmd in ("q", "quit"):
            stop_event.set()
            break
        elif cmd in [str(i) for i in range(7)]:
            active = int(cmd)
            print(f"  Active joint: {active}")
            _print_status(joylo, active)
        elif cmd == "f":
            joylo.flip_joint_sign(active)
            s = "+" if joylo.joint_signs[active] > 0 else "-"
            print(f"  J{active} sign -> {s}1")
            _print_status(joylo, active)
        elif cmd == "z":
            system.stop()
            sim_franka.send_joint_positions(home_qpos)
            print()
            print("  Sim frozen at home pose. Move the Joylo to match, then press Enter.")
            print(f"  Home: {np.array2string(home_qpos, precision=3, separator=', ')}")
            input("  Press Enter when ready... ")
            current = joylo.joint_positions
            for i in range(7):
                joylo.nudge_joint_offset(i, home_qpos[i] - current[i])
            print("  Offsets snapped!")
            _print_status(joylo, active)
            system.start_teleop()
        elif cmd == "+":
            joylo.nudge_joint_offset(active, COARSE_STEP)
            print(f"  J{active} offset -> {joylo.joint_offsets_rad[active]:+.3f} rad  (+{COARSE_STEP})")
            _print_status(joylo, active)
        elif cmd == "-":
            joylo.nudge_joint_offset(active, -COARSE_STEP)
            print(f"  J{active} offset -> {joylo.joint_offsets_rad[active]:+.3f} rad  (-{COARSE_STEP})")
            _print_status(joylo, active)
        elif cmd == "]":
            joylo.nudge_joint_offset(active, FINE_STEP)
            print(f"  J{active} offset -> {joylo.joint_offsets_rad[active]:+.3f} rad  (+{FINE_STEP})")
            _print_status(joylo, active)
        elif cmd == "[":
            joylo.nudge_joint_offset(active, -FINE_STEP)
            print(f"  J{active} offset -> {joylo.joint_offsets_rad[active]:+.3f} rad  (-{FINE_STEP})")
            _print_status(joylo, active)
        elif cmd == "r":
            old = joylo.joint_offsets_rad[active]
            joylo.nudge_joint_offset(active, -old)
            print(f"  J{active} offset -> 0.000 rad  (reset)")
            _print_status(joylo, active)
        elif cmd == "s":
            _save_config(config_path, joylo)
            print(f"  Saved to {config_path}")
        else:
            print("  Commands: 0-6 (select joint), f (flip sign), z (snap offsets),")
            print("           +/- (offset 0.1), ]/[ (offset 0.01), r (reset offset),")
            print("           s (save), q (quit)")


def main():
    parser = argparse.ArgumentParser(description="Sim teleop: Joylo drives MuJoCo Franka")
    parser.add_argument("--config", type=str, default=str(CONFIG_PATH),
                        help="Path to config.json from calibration")
    parser.add_argument("--rate", type=float, default=100.0,
                        help="Control loop rate in Hz")
    parser.add_argument("--alpha", type=float, default=0.95,
                        help="EMA smoothing factor (1.0 = no smoothing)")
    args = parser.parse_args()

    with open(args.config) as f:
        config = json.load(f)

    print(f"Loaded config from {args.config}")
    print(f"  5V port:  {config['port_5v']}")
    print(f"  12V port: {config['port_12v']}")

    sim_franka = SimFrankaInterface.create()
    model, data = sim_franka.model, sim_franka.data
    home_qpos = data.qpos[:7].copy()
    data.ctrl[:7] = home_qpos

    joylo = Joylo(
        port_5v=config["port_5v"],
        port_12v=config["port_12v"],
        joint_signs=np.array(config["joint_signs"]),
        joint_offsets_rad=np.array(config["joint_offsets_rad"]),
        gravity_comp_currents=np.array(config.get("gravity_comp_currents", [0]*7), dtype=int),
    )

    system = JoyloSystem(sim_franka, joylo,
                         control_rate_hz=args.rate,
                         smoothing_alpha=args.alpha)

    print()
    print("Teleop running. Move the Joylo and watch the sim Franka.")
    print()
    print("  0-6       select joint         f         flip sign")
    print("  z         snap all offsets     + / -     offset +/- 0.1 rad")
    print("  ] / [     offset +/- 0.01     r         reset offset")
    print("  s         save config          q         quit")
    print()

    system.start_teleop()

    stop_event = threading.Event()
    input_thread = threading.Thread(
        target=_input_loop,
        args=(joylo, system, sim_franka, home_qpos, args.config, stop_event),
        daemon=True,
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
