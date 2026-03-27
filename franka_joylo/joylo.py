"""Manages 7 Joylo motors across 2 DxlDriver instances."""

import threading

import numpy as np

from franka_joylo.constants import (
    DXL_TICKS_PER_REV,
    DXL_TO_RAD,
    DXL_ZERO_OFFSETS,
    FRANKA_JOINT_CENTER,
    JOINT_MAP,
    MODE_CURRENT,
    MODE_POSITION,
    MOTOR_IDS_5V,
    MOTOR_IDS_12V,
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
            matching the Joylo's zero pose.
        gravity_comp_currents: Array of shape (7,) — feedforward currents for
            gravity compensation in current mode. If None, zeros are used.
    """

    def __init__(
        self,
        port_5v: str,
        port_12v: str,
        motor_ids_5v: list[int] | None = None,
        motor_ids_12v: list[int] | None = None,
        baudrate: int = 1000000,
        joint_signs: np.ndarray | None = None,
        joint_offsets_rad: np.ndarray | None = None,
        gravity_comp_currents: np.ndarray | None = None,
    ):
        self.motor_ids_5v = motor_ids_5v or MOTOR_IDS_5V
        self.motor_ids_12v = motor_ids_12v or MOTOR_IDS_12V

        self._driver_5v = DxlDriver(port_5v, self.motor_ids_5v, baudrate)
        self._driver_12v = DxlDriver(port_12v, self.motor_ids_12v, baudrate)

        self._joint_signs = joint_signs if joint_signs is not None else np.ones(NUM_JOINTS)
        self._joint_offsets_rad = joint_offsets_rad if joint_offsets_rad is not None else np.zeros(NUM_JOINTS)
        self._gravity_comp_currents = gravity_comp_currents if gravity_comp_currents is not None else np.zeros(NUM_JOINTS, dtype=int)
        self._cal_lock = threading.Lock()

    # --- Public API ---

    @property
    def joint_positions(self) -> np.ndarray:
        """Read all 7 joint positions, return radians as shape (7,) array."""
        raw = self._read_raw_positions()
        with self._cal_lock:
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
        # Zero goal currents before enabling torque. Changing operating mode
        # resets Goal Current to Current Limit; enabling torque at that value
        # causes a current surge that sags the 5V rail and trips the input
        # voltage error on lower-power motors (e.g. XC330-T288).
        self._driver_5v.write_currents({mid: 0 for mid in self.motor_ids_5v})
        self._driver_12v.write_currents({mid: 0 for mid in self.motor_ids_12v})
        self._driver_5v.set_torque(True)
        self._driver_12v.set_torque(True)

    def command_positions(self, target_rad: np.ndarray) -> None:
        """Command joint positions in radians. Motors must be in position mode."""
        with self._cal_lock:
            raw = self._rad_to_dxl(target_rad)
        positions_5v, positions_12v = self._split_by_controller(raw)
        self._driver_5v.write_positions(positions_5v)
        self._driver_12v.write_positions(positions_12v)

    def command_currents(self, currents: np.ndarray) -> None:
        """Command raw goal currents. Motors must be in current mode."""
        currents_5v, currents_12v = self._split_by_controller(currents)
        self._driver_5v.write_currents(currents_5v)
        self._driver_12v.write_currents(currents_12v)

    def command_gravity_comp(self) -> None:
        """Send feedforward gravity compensation currents."""
        self.command_currents(self._gravity_comp_currents)

    @property
    def joint_signs(self) -> np.ndarray:
        """Current joint sign array (shape (7,), values +1 or -1)."""
        return self._joint_signs.copy()

    @property
    def joint_offsets_rad(self) -> np.ndarray:
        """Current joint offset array in radians (shape (7,))."""
        return self._joint_offsets_rad.copy()

    def flip_joint_sign(self, joint_idx: int) -> None:
        """Flip the sign (+1 <-> -1) for a single joint.

        Adjusts the offset so the current position is preserved —
        only the direction of future motion changes.
        """
        raw = self._read_raw_positions()
        with self._cal_lock:
            current_pos = self._dxl_to_rad(raw)[joint_idx]
            old_offset = self._joint_offsets_rad[joint_idx]
            self._joint_signs[joint_idx] *= -1
            self._joint_offsets_rad[joint_idx] = 2 * current_pos - old_offset

    def nudge_joint_offset(self, joint_idx: int, delta_rad: float) -> None:
        """Add delta_rad to a single joint's offset."""
        with self._cal_lock:
            self._joint_offsets_rad[joint_idx] += delta_rad

    def disable_torque(self) -> None:
        """Disable torque on all motors."""
        self._driver_5v.set_torque(False)
        self._driver_12v.set_torque(False)

    def close(self) -> None:
        """Disable torque and close both ports."""
        try:
            self._driver_5v.close()
        finally:
            self._driver_12v.close()

    # --- Private helpers ---

    def _read_raw_positions(self) -> np.ndarray:
        """Read raw DXL positions from both controllers, return shape (7,) ordered by joint index."""
        pos_5v = self._driver_5v.read_positions()
        pos_12v = self._driver_12v.read_positions()
        raw = np.empty(NUM_JOINTS, dtype=np.float64)
        for joint_idx, (controller, motor_id) in enumerate(JOINT_MAP):
            if controller == "5v":
                raw[joint_idx] = pos_5v[motor_id]
            else:
                raw[joint_idx] = pos_12v[motor_id]
        return raw

    def _dxl_to_rad(self, raw: np.ndarray) -> np.ndarray:
        """Convert raw DXL positions to radians, robust to encoder wrapping.

        Computes the angle then normalizes to within ±π of each joint's
        range center.  This handles multi-turn accumulation, reboots, and
        asymmetric DXL ranges (e.g. joint 5 whose operating delta exceeds
        half a revolution from DXL_ZERO_OFFSETS).
        """
        rad = (raw - DXL_ZERO_OFFSETS) * self._joint_signs * DXL_TO_RAD + self._joint_offsets_rad
        return FRANKA_JOINT_CENTER + np.mod(rad - FRANKA_JOINT_CENTER + np.pi, 2 * np.pi) - np.pi

    def _rad_to_dxl(self, rad: np.ndarray) -> np.ndarray:
        """Convert radians to absolute DXL positions (0-4095 range)."""
        delta = (rad - self._joint_offsets_rad) / self._joint_signs * RAD_TO_DXL
        return np.mod(DXL_ZERO_OFFSETS + delta, DXL_TICKS_PER_REV)

    def _split_by_controller(self, values: np.ndarray) -> tuple[dict[int, int], dict[int, int]]:
        """Split a (7,) array (ordered by joint index) into dicts for each controller."""
        dict_5v: dict[int, int] = {}
        dict_12v: dict[int, int] = {}
        for joint_idx, (controller, motor_id) in enumerate(JOINT_MAP):
            if controller == "5v":
                dict_5v[motor_id] = int(round(values[joint_idx]))
            else:
                dict_12v[motor_id] = int(round(values[joint_idx]))
        return dict_5v, dict_12v
