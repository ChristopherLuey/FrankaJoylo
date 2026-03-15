# FrankaJoylo

Teleoperate a Franka Emika Panda (7-DOF) using a Joylo — a custom 7-DOF controller built from Dynamixel motors.

## Hardware

The Joylo has 7 Dynamixel motors mapping 1:1 to Franka's 7 joints, split across two USB controllers by voltage. Motor IDs overlap across buses — `JOINT_MAP` in `constants.py` routes each Franka joint to the correct (controller, motor_id) pair.

| Controller | Voltage | Motor Model | Motor IDs | Symlink |
|-----------|---------|-------------|-----------|---------|
| Small | 5V | XL330 | 0, 1, 2, 3 | `/dev/franka_small` |
| Large | 12V | XL430 | 0, 1, 2 | `/dev/franka_large` |

All motors use Dynamixel Protocol 2.0 at **1000000** baud.

## Installation

```bash
conda env create -f environment.yml
conda activate franka_joylo
pip install -e .
```

Or just pip:

```bash
pip install -e .
pip install dynamixel-sdk mujoco
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

## Usage

Once calibrated, use the package in your own code. You must provide a concrete `FrankaInterface` for your Franka setup:

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

# Tracking: Franka leads, Joylo follows
system.start_tracking()

# Teleop: Joylo leads, Franka follows
system.start_teleop()

# Stop and clean up
system.close()
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

These values are stored in `config.json` and loaded at runtime.

## Package Structure

```
franka_joylo/
├── __init__.py            # Re-exports: FrankaInterface, Joylo, JoyloSystem
├── constants.py           # Register addresses, JOINT_MAP, DXL_ZERO_OFFSETS, unit conversions
├── dxl_driver.py          # Low-level DXL SDK wrapper (one instance per USB port)
├── joylo.py               # Manages 7 motors across 2 DxlDrivers via JOINT_MAP
├── franka_interface.py    # Abstract base class for Franka communication
├── sim.py                 # MuJoCo SimFrankaInterface for testing without hardware
└── joylo_system.py        # Top-level orchestrator: tracking + teleop modes
```

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
