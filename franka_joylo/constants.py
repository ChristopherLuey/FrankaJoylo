"""Dynamixel Protocol 2.0 register addresses, motor config, and unit conversions."""

from math import pi

import numpy as np

# --- Register addresses (Protocol 2.0) ---
ADDR_OPERATING_MODE = 11    # 1 byte
ADDR_TORQUE_ENABLE = 64     # 1 byte
ADDR_GOAL_CURRENT = 102     # 2 bytes
ADDR_GOAL_POSITION = 116    # 4 bytes
ADDR_PRESENT_CURRENT = 126  # 2 bytes
ADDR_PRESENT_POSITION = 132 # 4 bytes

# --- Operating modes ---
MODE_CURRENT = 0
MODE_POSITION = 3

# --- Motor ID groups by controller ---
MOTOR_IDS_5V = [0, 1, 2, 3]   # 5V, XL330
MOTOR_IDS_12V = [0, 1, 2]     # 12V, XL430

# --- Joint-to-controller mapping ---
# Each entry is (controller, motor_id) for joint index 0-6
JOINT_MAP: list[tuple[str, int]] = [
    ("12v", 0), ("12v", 1), ("5v", 0), ("12v", 2),
    ("5v", 1),  ("5v", 2),  ("5v", 3),
]

# --- DXL zero-point offsets (raw position when Franka joint is at 0 rad) ---
DXL_ZERO_OFFSETS = np.array([3072, 1024, 0, 1024, 2048, 1024, 0], dtype=np.float64)

# --- Unit conversion ---
# XL330 and XL430 both have 4096 steps per revolution
DXL_TO_RAD = pi / 2048.0
RAD_TO_DXL = 2048.0 / pi

# --- Franka Panda joint position limits (radians) ---
FRANKA_JOINT_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
FRANKA_JOINT_MAX = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])

# --- Franka Panda home pose (matches MJCF keyframe) ---
FRANKA_HOME = np.array([0.0, 0.0, 0.0, -pi / 2, 0.0, pi / 2, -pi / 4])


# --- Byte helpers ---
def dxl_lobyte(value: int) -> int:
    return value & 0xFF


def dxl_hibyte(value: int) -> int:
    return (value >> 8) & 0xFF


def dxl_loword(value: int) -> int:
    return value & 0xFFFF


def dxl_hiword(value: int) -> int:
    return (value >> 16) & 0xFFFF
