# FrankaJoylo

Teleoperate a Franka Emika Panda (7-DOF) using a Joylo — a custom 7-DOF controller built from Dynamixel motors.

## Architecture

```
Workstation (your PC)              NUC (mini-PC)              Franka
┌─────────────────────┐      ┌──────────────────┐      ┌──────────────┐
│  FrankaJoylo Python  │      │  auto_arm.sh     │      │  Control Box │
│  (JoyloSystem)       │─ZMQ──│  franka-interface │─1kHz─│  + Arm       │
│                      │      │  (libfranka)      │      │              │
│  Joylo USB serial    │      └──────────────────┘      │  Franka Desk │
│  (Dynamixel motors)  │           via ethernet          │  (browser UI)│
└─────────────────────┘                                  └──────────────┘
```

Three hardware components work together:

1. **Workstation** — runs your Python code, reads the Joylo via USB, sends commands to the NUC via ZMQ.
2. **NUC** — a mini-PC running a real-time kernel. Runs `franka-interface` which sends 1kHz commands to the Franka control box via libfranka.
3. **Franka arm + control box** — the physical robot. Managed via Franka Desk (a browser UI) for unlocking joints and activating FCI.

## Hardware

The Joylo has 7 Dynamixel motors mapping 1:1 to Franka's 7 joints, split across two USB controllers by voltage. Motor IDs overlap across buses — `JOINT_MAP` in `constants.py` routes each Franka joint to the correct (controller, motor_id) pair.

| Controller | Voltage | Motor Model | Motor IDs | Symlink |
|-----------|---------|-------------|-----------|---------|
| Small | 5V | XL330 | 0, 1, 2, 3 | `/dev/franka_small` |
| Large | 12V | XL430 | 0, 1, 2 | `/dev/franka_large` |

Joint-to-controller mapping (defined in `constants.py`):

```
Joint 0 → (12V, motor 0)    Joint 4 → (5V, motor 1)
Joint 1 → (12V, motor 1)    Joint 5 → (5V, motor 2)
Joint 2 → (5V, motor 0)     Joint 6 → (5V, motor 3)
Joint 3 → (12V, motor 2)
```

All motors use Dynamixel Protocol 2.0 at **1000000** baud.

## Installation

### Sim only (Joylo + MuJoCo, no real robot)

```bash
conda env create -f environment.yml
conda activate franka_joylo
```

This creates a `franka_joylo` env with Python 3.10, numpy, dynamixel-sdk, mujoco >= 3.0, and this package (editable install).

### Sim + real robot (adds deoxys)

After creating the env above, also install the [deoxys](https://github.com/UT-Austin-RPL/deoxys_control) Python package:

```bash
conda activate franka_joylo
cd /home/eeg/deoxys_control/deoxys
pip install -e .
```

Note: `setup.py` is inside `deoxys_control/deoxys/`, not the top-level `deoxys_control/`.

Alternatively, if you already have a conda env with deoxys (e.g., the `dp` env), you can add FrankaJoylo to it instead:

```bash
conda activate dp
cd /home/eeg/yingke/FrankaJoylo
pip install -e ".[sim]"
```

### Verify

```bash
python -c "from franka_joylo import Joylo, JoyloSystem, FrankaInterface; print('Core OK')"
python -c "from franka_joylo.sim import SimFrankaInterface; print('Sim OK')"
python -c "from franka_joylo.deoxys_franka import DeoxysFrankaInterface; print('Deoxys OK')"
```

## Setup

Run the interactive setup script to configure everything:

```bash
./setup.sh
```

This walks you through three steps:

### Step 1: USB Port Detection

Detects which `/dev/ttyUSB*` port belongs to which controller by having you unplug and replug the 5V controller. Creates persistent symlinks (`/dev/franka_small`, `/dev/franka_large`) via udev rules so ports survive reboots.

If the symlinks already exist, this step is skipped.

**Important:** If you have other USB-serial devices plugged in (cameras, other adapters), there may be more than two `/dev/ttyUSB*` ports. Make sure you unplug the correct controller (the 5V / XL330 one). If the ports end up wrong, you can verify with:

```bash
python examples/calibrate.py --scan-only
```

This pings all motors on both buses. You should see:
- `franka_small`: [0, 1, 2, 3] (4 motors on the 5V bus)
- `franka_large`: [0, 1, 2] (3 motors on the 12V bus)

If the counts are wrong or ports are swapped, fix the symlinks manually:

```bash
sudo ln -sf /dev/ttyUSBX /dev/franka_small   # the port with 4 motors
sudo ln -sf /dev/ttyUSBY /dev/franka_large   # the port with 3 motors
```

Also fix the USB latency timers for reliable communication:

```bash
sudo chmod a+w /sys/bus/usb-serial/devices/ttyUSBX/latency_timer
sudo chmod a+w /sys/bus/usb-serial/devices/ttyUSBY/latency_timer
```

### Step 2: Calibration

Pings all motors on both buses, verifies connectivity, reads current positions, and writes `config.json` with default signs and offsets.

If `config.json` already exists, you'll be asked before overwriting — so tuned values aren't lost.

### Step 3: MuJoCo Sim Teleop

Launches a MuJoCo viewer with the Franka model. Move the Joylo and watch the sim robot mirror your movements. Use the terminal commands to tune:

| Command | Action |
|---------|--------|
| `0`-`6` | Select active joint |
| `f` | Flip sign of active joint |
| `z` | Snap all offsets to home pose |
| `+` / `-` | Nudge offset by +/- 0.1 rad |
| `]` / `[` | Nudge offset by +/- 0.01 rad |
| `r` | Reset active joint offset to 0 |
| `s` | Save config to `config.json` |
| `q` | Quit |

## Operating Modes

| Mode | Who Leads | Behavior |
|------|-----------|----------|
| `tracking` | Franka leads | Reads Franka joint angles, commands Joylo to follow (position mode). The Franka does NOT move. |
| `teleop` | Joylo leads | Joylo enters current mode (free-spinning), you move it by hand, Franka follows with EMA smoothing. |
| `takeover` (default) | Tracking → Teleop | Starts in tracking. Press Enter to switch to teleop. Press Enter again to stop. |

**Takeover** is the recommended mode: the Joylo first syncs to the robot's pose (tracking), then you take control (teleop).

## Real Robot Teleop (deoxys)

### Safety

- **Always hold the E-stop** when your script is running.
- **Always lock joints** in Franka Desk when you're not using the robot, even for a short break.
- If the robot is near joint limits or in a weird configuration, kill your script first, go into Programming mode in Franka Desk, and manually move the robot to a safe pose.
- Always reset to a safe configuration and **lock joints** before powering off.

### Robot LED Colors

| Color | Meaning |
|-------|---------|
| Yellow | Booting / initializing |
| Blue | Joints unlocked, ready, FCI not yet active |
| Green | FCI active, autonomous mode, accepting commands |
| White | Programming mode (arm is backdrivable) |
| Red / pink | Error or fault (check Franka Desk) |
| Flashing white | Shutting down |

### Network Topology

| Device | IP | Name |
|--------|----|------|
| Workstation (this PC) | `172.16.0.1` | rosie |
| Franka control box | `172.16.0.2` | — |
| NUC | `172.16.0.3` | charmander |

All connected via ethernet through a network switch. ZMQ ports: arm `5555`/`5556`, gripper `5557`/`5558`. Config file: `charmander.yml` (inside the deoxys install at `/home/eeg/deoxys_control/deoxys/config/charmander.yml`).

### Startup Procedure

#### Step 1: Check physical connections

1. **Control box** is connected to: wall power, the Franka arm (thick cable to arm base), the E-stop button (cable into control box), and the network switch (ethernet).
2. **NUC** is connected to: its power adapter and the network switch (ethernet).
3. **Workstation** is connected to: the network switch (ethernet) and both Joylo USB cables.
4. Verify your workstation ethernet is set to IP `172.16.0.1`, netmask `255.255.255.0`.

#### Step 2: Power on

5. **Control box** — flip the rocker switch on the **back** of the box. The arm LED ring turns **yellow** while booting.
6. **NUC** — press the power button on **top** of the NUC (if not already on).

#### Step 3: Franka Desk (browser)

7. Wait for the arm LED to stop being yellow.
8. Open **http://172.16.0.2/desk** in your browser.
   - Franka Desk login — username: `svl-fr3`, password: `learning_xxx`
   - If it says "request control", click it and press the small **circle button** on the robot base.
9. **Unlock joints** — click the lock/unlock button on the **left side** of the Franka Desk UI. LED turns **blue**.
10. **Activate FCI** — click the **top-right dropdown menu** → **"Activate FCI"**. LED turns **green**. The robot is now accepting commands.
    - (Optional) To manually position the robot first: click **"Programming mode"** on the left side. LED turns **white** (arm is backdrivable). Push it to a safe pose. Then activate FCI again.

#### Step 4: Start deoxys on the NUC

11. SSH into the NUC from your workstation:

```bash
ssh 172.16.0.3
# User: franka3 (configured in ~/.ssh/config)
# Password: learning
```

12. On the NUC, start the arm interface in tmux:

```bash
tmux
cd deoxys_control/deoxys
./auto_scripts/auto_arm.sh config/charmander.yml
```

You should see output about performance mode and then the interface starting. If you see **"No control callback / core dumped"**, the NUC is not in the RT kernel — reboot the NUC and press the **DOWN arrow key** repeatedly during boot to select the RT kernel from the GRUB menu.

13. (Optional) Open a second tmux pane (`Ctrl+B` then `%`) and start the gripper interface. FrankaJoylo doesn't use the gripper, so this is optional:

```bash
./auto_scripts/auto_gripper.sh config/charmander.yml
```

Leave these scripts running — they auto-restart if they crash.

#### Step 5: Activate the conda environment

14. On your workstation:

```bash
conda activate franka_joylo
cd /home/eeg/yingke/FrankaJoylo
```

#### Step 6: Verify connection (safe — robot does NOT move)

15. Run tracking mode to test the connection without moving the robot:

```bash
python examples/real_teleop.py --mode tracking
```

What to expect:
- It prints `Connecting to Franka via deoxys...`
- It prints `Waiting for robot state...` then `OK` with 7 joint positions in radians.
- It says `Tracking mode: Franka leads, Joylo follows.`
- The **Joylo motors snap** to match the Franka's current pose. **The Franka does NOT move.**
- If `Waiting for robot state...` hangs forever — `auto_arm.sh` is not running on the NUC, or the network is wrong. Try `ping 172.16.0.3` to check.
- Press **Enter** to stop. Press **Ctrl+C** if Enter doesn't work.

If this worked, your connection is good.

#### Step 7: Run teleop (the robot WILL move)

16. **Put your hand on the E-stop. This is not optional.**

17. Run takeover mode:

```bash
python examples/real_teleop.py --mode takeover
```

What happens step by step:
- Same connection as above.
- **Tracking starts** — the Joylo motors snap to the Franka's current pose. The Franka does NOT move yet.
- **Press Enter** — the system transitions to teleop. The Joylo becomes loose (free-spinning). The Franka switches to compliant gains.
- **Move the Joylo SLOWLY at first.** The Franka will gently follow. If anything looks wrong, **hit the E-stop** or press **Enter**.
- **Press Enter again** to stop. The Franka holds its last position.

18. If the robot moves in the wrong direction or calibration looks off, press Enter or Ctrl+C immediately. Go back and re-tune in sim (`python examples/sim_teleop.py`).

Options: `--rate` (Hz, default 100), `--alpha` (EMA smoothing, default 0.95), `--config` (path to config.json).

### Shutdown Procedure

19. **Stop your script** — press Enter or Ctrl+C.
20. In Franka Desk, click **"Programming mode"** on the left side. LED turns **white**. Manually push the robot to a safe home-like configuration (roughly centered, not near joint limits).
21. **Lock the joints** — click the lock button in Franka Desk.
22. **Deactivate FCI** — top-right dropdown menu.
23. Click **"Shut down"** in the top-right dropdown. Wait for "Finished shutting down". LED flashes **white**.
24. **Flip the power switch** on the back of the control box to OFF. Wait at least **~30 seconds** before turning back on.
25. NUC scripts can keep running (they just idle), or kill them and power off the NUC if done for the day.

### Tuning compliant gains

In teleop mode the Franka uses low-impedance joint control so it's compliant. The default gains (`kp=[10,10,10,10,5,5,2]`, `kd=[6,6,6,6,4,4,2]`) may need tuning on your robot. You can override them in code:

```python
from franka_joylo.deoxys_franka import DeoxysFrankaInterface

franka = DeoxysFrankaInterface(
    compliant_kp=[8, 8, 8, 8, 4, 4, 2],
    compliant_kd=[5, 5, 5, 5, 3, 3, 1],
)
```

## How Gravity Compensation Works

Gravity compensation operates on **both** the Joylo and the Franka during teleop, but the mechanisms are different.

### Joylo side: current mode with zero torque

When teleop starts, `JoyloSystem.start_teleop()` does three things to the Joylo:

1. `disable_torque()` — turns off all 7 motors
2. `set_current_mode()` — switches to current control (mode 0), re-enables torque
3. Every loop iteration (100 Hz): `command_gravity_comp()` sends `gravity_comp_currents` to all motors

In current control mode, you command torque (via current) rather than position. With `gravity_comp_currents = [0,0,0,0,0,0,0]`, the motors produce zero torque — free-spinning. You move the Joylo by hand while the encoders report the position.

The `gravity_comp_currents` field exists for heavier Joylo designs where gravity would pull the arm down. You would set per-joint feedforward currents to counteract gravity. With a lightweight Joylo, zeros work fine.

### Franka side: low-impedance joint control

The Franka uses a **JOINT_IMPEDANCE** controller, not true gravity compensation. `start_gravity_comp()` switches from stiff to compliant gains:

- **Stiff** (tracking): `kp=[100,100,100,100,100,100,100]`, `kd=[10,10,10,10,10,10,10]`
- **Compliant** (teleop): `kp=[10,10,10,10,5,5,2]`, `kd=[6,6,6,6,4,4,2]`

The impedance controller applies: `torque = kp * (cmd - actual) + kd * (cmd_vel - actual_vel)`.

With high kp the robot rigidly tracks positions. With low kp it only weakly follows — it feels "soft." The Franka's **control box always runs its own internal gravity compensation at the firmware level**, so the arm won't collapse under its own weight even with low impedance gains.

The transition is jerk-free: `start_gravity_comp()` immediately sends the current position with the new gains.

### The full teleop loop (100 Hz)

Each iteration of `_teleop_loop()`:

1. Send zero current to Joylo motors (keep them free-spinning)
2. Read Joylo joint positions (where the human moved it)
3. Apply EMA smoothing: `smoothed = alpha * new + (1 - alpha) * smoothed`
4. Send smoothed positions to Franka with compliant gains
5. Read back Franka's actual joint positions

The default `alpha=0.95` means 95% weight on the new reading — very responsive. Lower alpha = more smoothing = smoother but slower.

### In simulation

`start_gravity_comp()` and `stop_gravity_comp()` are no-ops in `SimFrankaInterface`. MuJoCo position actuators always rigidly track commanded positions.

## Custom FrankaInterface

For other Franka setups (not deoxys), subclass `FrankaInterface`:

```python
import json
import numpy as np
from franka_joylo import FrankaInterface, Joylo, JoyloSystem

class MyFranka(FrankaInterface):
    def read_joint_positions(self) -> np.ndarray: ...
    def send_joint_positions(self, positions: np.ndarray) -> None: ...
    def start_gravity_comp(self) -> None: ...
    def stop_gravity_comp(self) -> None: ...

with open("config.json") as f:
    config = json.load(f)

franka = MyFranka()
joylo = Joylo(
    port_5v=config["port_5v"],
    port_12v=config["port_12v"],
    baudrate=config["baudrate"],
    joint_signs=np.array(config["joint_signs"]),
    joint_offsets_rad=np.array(config["joint_offsets_rad"]),
    gravity_comp_currents=np.array(config["gravity_comp_currents"]),
)
system = JoyloSystem(franka, joylo)

system.start_tracking()   # Franka leads, Joylo follows
system.start_teleop()     # Joylo leads, Franka follows
system.close()            # Stop and clean up
```

See `examples/` for full working scripts.

## Calibration Details

Joint positions are computed from raw Dynamixel values using hardcoded zero-point offsets (`DXL_ZERO_OFFSETS` in `constants.py`):

```
rad = (raw - DXL_ZERO_OFFSET) * joint_sign * DXL_TO_RAD + joint_offset
```

- `DXL_ZERO_OFFSETS` — raw DXL position when the corresponding Franka joint is at 0 rad
- `joint_signs` — +1 or -1 per joint (corrects direction mismatches)
- `joint_offsets_rad` — per-joint offset in radians (tuned in the sim viewer)
- `DXL_TO_RAD` = pi / 2048 (4096 steps per revolution for both XL330 and XL430)

These values are stored in `config.json` and loaded at runtime.

## Configuration

`config.json` fields:

| Field | Default | Description |
|-------|---------|-------------|
| `port_5v` | `/dev/franka_small` | USB port for the 5V controller (XL330) |
| `port_12v` | `/dev/franka_large` | USB port for the 12V controller (XL430) |
| `baudrate` | 1000000 | Dynamixel serial baudrate |
| `joint_signs` | `[1,-1,1,1,1,1,1]` | Direction mapping per joint (+/-1) |
| `joint_offsets_rad` | all 0 | Per-joint offset in radians |
| `gravity_comp_currents` | all 0 | Feedforward currents for gravity compensation |

## Package Structure

```
franka_joylo/
├── __init__.py            # Re-exports: FrankaInterface, Joylo, JoyloSystem, (DeoxysFrankaInterface)
├── constants.py           # Register addresses, JOINT_MAP, DXL_ZERO_OFFSETS, unit conversions
├── dxl_driver.py          # Low-level DXL SDK wrapper (one instance per USB port)
├── joylo.py               # Manages 7 motors across 2 DxlDrivers via JOINT_MAP
├── franka_interface.py    # Abstract base class for Franka communication
├── deoxys_franka.py       # Adapter for deoxys driver (optional, requires deoxys)
├── sim.py                 # MuJoCo SimFrankaInterface for testing without hardware
└── joylo_system.py        # Top-level orchestrator: tracking + teleop modes
```

## Troubleshooting

- **"Waiting for robot state..." hangs** — deoxys on the NUC isn't running or ZMQ can't connect. Check that `auto_arm.sh` is running and the network is configured.
- **"No control callback / core dumped" on NUC** — NUC is not booted into the RT kernel. Reboot and select the RT kernel.
- **"[TxRxResult] There is no status packet!"** — a Dynamixel motor didn't respond. Check USB connections, run `calibrate.py --scan-only` to see which motors respond. Fix latency timers with `sudo chmod a+w /sys/bus/usb-serial/devices/ttyUSBX/latency_timer`.
- **Motor scan shows wrong counts** — the USB port symlinks are pointing to the wrong devices. Run `calibrate.py --scan-only` with different `--port-5v` / `--port-12v` arguments to find the right ports, then fix the symlinks.
- **Ports swapped** — `calibrate.py` will hint if the 5V and 12V ports are swapped. Re-run `setup.sh` or fix symlinks manually.
- **`zmq.error.ZMQError: Address already in use`** — a previous deoxys process is still running. Use `sudo netstat -ltnp` to find the PID on port 5555/5556 and `kill -9 <PID>`.
- **Joint limit violation** — do NOT try to recover in programming mode. Use the External Enabling Device (came with the robot), connect it to the robot base, press gently, click "unlock joint" in Franka Desk, then hold "+/-" to move the joint out of violation.
- **Latency timer warnings** — the default FTDI latency is 16ms which can cause communication failures. Fix with `sudo chmod a+w /sys/bus/usb-serial/devices/ttyUSBX/latency_timer` for each Joylo port.

## Quick Reference

```bash
# --- Environment setup ---
conda env create -f environment.yml              # create franka_joylo env
conda activate franka_joylo
cd /home/eeg/deoxys_control/deoxys && pip install -e .   # add deoxys for real robot

# --- Joylo setup (USB ports + calibration + sim) ---
./setup.sh                                       # full interactive setup
python examples/calibrate.py                     # calibration only
python examples/calibrate.py --scan-only         # just ping motors (debug)

# --- Sim teleop (MuJoCo, no real robot needed) ---
python examples/sim_teleop.py
python examples/sim_teleop.py --rate 50 --alpha 0.8

# --- Real robot startup (do these in order) ---
# 1. Power on control box (switch on back) and NUC (button on top)
# 2. Franka Desk: http://172.16.0.2/desk → unlock joints → activate FCI
# 3. On the NUC:
ssh 172.16.0.3                                   # user: franka3, password: learning
tmux
cd deoxys_control/deoxys
./auto_scripts/auto_arm.sh config/charmander.yml # leave running in tmux

# 4. On the workstation:
conda activate franka_joylo
cd /home/eeg/yingke/FrankaJoylo
python examples/real_teleop.py --mode tracking   # verify (robot does NOT move)
python examples/real_teleop.py --mode takeover   # tracking → Enter → teleop

# --- Shutdown ---
# 1. Stop script (Enter or Ctrl+C)
# 2. Franka Desk: programming mode → move to safe pose → lock joints
# 3. Franka Desk: deactivate FCI → shut down
# 4. Power off control box (switch on back, wait 30s before restarting)
```
