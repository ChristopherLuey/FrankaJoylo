# FrankaJoylo

Teleoperate a Franka Emika Panda (7-DOF) using a Joylo — a custom 7-DOF controller built from Dynamixel motors.

## Modes

- **Tracking** — Franka leads, Joylo follows (position control)
- **Teleop** — Joylo leads, Franka follows (current control with gravity compensation)
- **Takeover** — Starts in tracking, auto-transitions to teleop via a trigger function

## Hardware

The Joylo has 7 Dynamixel motors (IDs 0–6) mapping 1:1 to Franka's 7 joints, split across two USB controllers by voltage:

| Controller | Voltage | Motor Model | Motor IDs |
|-----------|---------|-------------|-----------|
| Controller 1 | 5V | XL330 | 2, 4, 5, 6 |
| Controller 2 | 12V | XL430 | 0, 1, 3 |

All motors use Dynamixel Protocol 2.0 at 57600 baud (configurable).

> **Tip:** `/dev/ttyUSB*` ports can swap on reboot. Use udev rules for stable symlinks.

## Installation

```bash
conda env create -f environment.yml
conda activate franka_joylo
```

Or with pip:

```bash
pip install -e .
```

## Usage

You must provide a concrete `FrankaInterface` implementation for your Franka setup:

```python
import numpy as np
from franka_joylo import FrankaInterface, Joylo, JoyloSystem

class MyFranka(FrankaInterface):
    def read_joint_positions(self) -> np.ndarray: ...
    def send_joint_positions(self, positions: np.ndarray) -> None: ...
    def start_gravity_comp(self) -> None: ...
    def stop_gravity_comp(self) -> None: ...

franka = MyFranka()
joylo = Joylo(
    port_5v="/dev/ttyUSB0",
    port_12v="/dev/ttyUSB1",
    joint_offsets_rad=franka.read_joint_positions(),
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

## Calibration

At startup, the Joylo records the raw Dynamixel position of each motor. You provide `joint_offsets_rad` — the Franka's joint angles (radians) matching the Joylo's startup pose — and optionally `joint_signs` (±1 per joint) for direction mapping.

```
dxl_to_rad(raw) = (raw - dxl_init) * joint_sign * DXL_TO_RAD + joint_offset
```

Ensure the Joylo is in a known pose when you create the `Joylo` instance.

## Package Structure

```
franka_joylo/
├── __init__.py            # Re-exports: FrankaInterface, Joylo, JoyloSystem
├── constants.py           # DXL register addresses, motor config, unit conversions
├── dxl_driver.py          # Low-level DXL SDK wrapper (one instance per USB port)
├── joylo.py               # Manages 7 motors across 2 DxlDrivers
├── franka_interface.py    # Abstract base class for Franka communication
└── joylo_system.py        # Top-level orchestrator: tracking + teleop modes
```

## Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `baudrate` | 57600 | Dynamixel serial baudrate |
| `control_rate_hz` | 100.0 | Control loop frequency |
| `smoothing_alpha` | 0.95 | EMA smoothing in teleop (1.0 = no smoothing) |
| `joint_signs` | all +1 | Direction mapping per joint (±1) |
| `gravity_comp_currents` | all 0 | Feedforward currents for gravity compensation |
