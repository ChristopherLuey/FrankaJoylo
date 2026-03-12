"""Dynamixel Protocol 2.0 register addresses, motor config, and unit conversions."""

from math import pi

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
CONTROLLER_1_MOTOR_IDS = [2, 4, 5, 6]  # 5V, XL330
CONTROLLER_2_MOTOR_IDS = [0, 1, 3]     # 12V, XL430

# --- Unit conversion ---
# XL330 and XL430 both have 4096 steps per revolution
DXL_TO_RAD = pi / 2048.0
RAD_TO_DXL = 2048.0 / pi


# --- Byte helpers ---
def dxl_lobyte(value: int) -> int:
    return value & 0xFF


def dxl_hibyte(value: int) -> int:
    return (value >> 8) & 0xFF


def dxl_loword(value: int) -> int:
    return value & 0xFFFF


def dxl_hiword(value: int) -> int:
    return (value >> 16) & 0xFFFF
