"""Manages 7 Joylo motors across 2 DxlDriver instances."""

import numpy as np

from franka_joylo.constants import (
    CONTROLLER_1_MOTOR_IDS,
    CONTROLLER_2_MOTOR_IDS,
    DXL_TO_RAD,
    MODE_CURRENT,
    MODE_POSITION,
    RAD_TO_DXL,
)
from franka_joylo.dxl_driver import DxlDriver

NUM_JOINTS = 7


class Joylo:
    """High-level interface for the 7-DOF Joylo controller.

    Args:
        port_5v: USB port for the 5V controller (XL330 motors).
        port_12v: USB port for the 12V controller (XL430 motors).
        motor_ids_5v: Motor IDs on the 5V bus.
        motor_ids_12v: Motor IDs on the 12V bus.
        baudrate: Dynamixel baudrate.
        joint_signs: Array of shape (7,) with +1 or -1 per joint for direction mapping.
        joint_offsets_rad: Array of shape (7,) — Franka joint angles (radians)
            matching the Joylo's startup pose.
        gravity_comp_currents: Array of shape (7,) — feedforward currents for
            gravity compensation in current mode. If None, zeros are used.
    """

    def __init__(
        self,
        port_5v: str,
        port_12v: str,
        motor_ids_5v: list[int] | None = None,
        motor_ids_12v: list[int] | None = None,
        baudrate: int = 57600,
        joint_signs: np.ndarray | None = None,
        joint_offsets_rad: np.ndarray | None = None,
        gravity_comp_currents: np.ndarray | None = None,
    ):
        self.motor_ids_5v = motor_ids_5v or CONTROLLER_1_MOTOR_IDS
        self.motor_ids_12v = motor_ids_12v or CONTROLLER_2_MOTOR_IDS
        self._all_motor_ids = sorted(self.motor_ids_5v + self.motor_ids_12v)
        assert len(self._all_motor_ids) == NUM_JOINTS

        self._driver_5v = DxlDriver(port_5v, self.motor_ids_5v, baudrate)
        self._driver_12v = DxlDriver(port_12v, self.motor_ids_12v, baudrate)

        self._joint_signs = joint_signs if joint_signs is not None else np.ones(NUM_JOINTS)
        self._joint_offsets_rad = joint_offsets_rad if joint_offsets_rad is not None else np.zeros(NUM_JOINTS)
        self._gravity_comp_currents = gravity_comp_currents if gravity_comp_currents is not None else np.zeros(NUM_JOINTS, dtype=int)

        # Calibration: record raw DXL positions at startup
        self._dxl_init = self._read_raw_positions()

    # --- Public API ---

    @property
    def joint_positions(self) -> np.ndarray:
        """Read all 7 joint positions, return radians as shape (7,) array."""
        raw = self._read_raw_positions()
        return self._dxl_to_rad(raw)

    def set_position_mode(self) -> None:
        """Switch all motors to position control mode and enable torque."""
        self._driver_5v.set_operating_mode(MODE_POSITION)
        self._driver_12v.set_operating_mode(MODE_POSITION)
        self._driver_5v.set_torque(True)
        self._driver_12v.set_torque(True)

    def set_current_mode(self) -> None:
        """Switch all motors to current control mode and enable torque."""
        self._driver_5v.set_operating_mode(MODE_CURRENT)
        self._driver_12v.set_operating_mode(MODE_CURRENT)
        self._driver_5v.set_torque(True)
        self._driver_12v.set_torque(True)

    def command_positions(self, target_rad: np.ndarray) -> None:
        """Command joint positions in radians. Motors must be in position mode."""
        raw = self._rad_to_dxl(target_rad)
        positions_5v, positions_12v = self._split_by_controller(raw)
        self._driver_5v.write_positions(positions_5v)
        self._driver_12v.write_positions(positions_12v)

    def command_currents(self, currents: np.ndarray) -> None:
        """Command raw goal currents. Motors must be in current mode."""
        current_dict = {mid: int(currents[i]) for i, mid in enumerate(self._all_motor_ids)}
        currents_5v = {mid: current_dict[mid] for mid in self.motor_ids_5v}
        currents_12v = {mid: current_dict[mid] for mid in self.motor_ids_12v}
        self._driver_5v.write_currents(currents_5v)
        self._driver_12v.write_currents(currents_12v)

    def command_gravity_comp(self) -> None:
        """Send feedforward gravity compensation currents."""
        self.command_currents(self._gravity_comp_currents)

    def disable_torque(self) -> None:
        """Disable torque on all motors."""
        self._driver_5v.set_torque(False)
        self._driver_12v.set_torque(False)

    def close(self) -> None:
        """Disable torque and close both ports."""
        self._driver_5v.close()
        self._driver_12v.close()

    # --- Private helpers ---

    def _read_raw_positions(self) -> np.ndarray:
        """Read raw DXL positions from both controllers, return shape (7,) ordered by motor ID."""
        pos_5v = self._driver_5v.read_positions()
        pos_12v = self._driver_12v.read_positions()
        merged = {**pos_5v, **pos_12v}
        return np.array([merged[mid] for mid in self._all_motor_ids], dtype=np.float64)

    def _dxl_to_rad(self, raw: np.ndarray) -> np.ndarray:
        """Convert raw DXL positions to radians using calibration."""
        return (raw - self._dxl_init) * self._joint_signs * DXL_TO_RAD + self._joint_offsets_rad

    def _rad_to_dxl(self, rad: np.ndarray) -> np.ndarray:
        """Convert radians to raw DXL positions using calibration."""
        return (rad - self._joint_offsets_rad) / self._joint_signs * RAD_TO_DXL + self._dxl_init

    def _split_by_controller(self, raw: np.ndarray) -> tuple[dict[int, int], dict[int, int]]:
        """Split a (7,) array (ordered by motor ID) into dicts for each controller."""
        all_dict = {mid: int(raw[i]) for i, mid in enumerate(self._all_motor_ids)}
        pos_5v = {mid: all_dict[mid] for mid in self.motor_ids_5v}
        pos_12v = {mid: all_dict[mid] for mid in self.motor_ids_12v}
        return pos_5v, pos_12v
