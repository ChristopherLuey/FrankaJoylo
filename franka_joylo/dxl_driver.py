"""Low-level Dynamixel SDK wrapper — one instance per USB port."""

import threading

import numpy as np
from dynamixel_sdk import (
    GroupSyncRead,
    GroupSyncWrite,
    PacketHandler,
    PortHandler,
)

from franka_joylo.constants import (
    ADDR_GOAL_CURRENT,
    ADDR_GOAL_POSITION,
    ADDR_OPERATING_MODE,
    ADDR_PRESENT_POSITION,
    ADDR_TORQUE_ENABLE,
    dxl_lobyte,
    dxl_hibyte,
    dxl_loword,
    dxl_hiword,
)

PROTOCOL_VERSION = 2.0


class DxlDriver:
    """Manages a single Dynamixel bus (one USB port, multiple motors)."""

    def __init__(self, port: str, motor_ids: list[int], baudrate: int = 57600):
        self.port = port
        self.motor_ids = motor_ids
        self.baudrate = baudrate
        self._lock = threading.Lock()

        self._port_handler = PortHandler(port)
        self._packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self._port_handler.openPort():
            raise RuntimeError(f"Failed to open port {port}")
        if not self._port_handler.setBaudRate(baudrate):
            raise RuntimeError(f"Failed to set baudrate {baudrate} on {port}")

    def set_operating_mode(self, mode: int) -> None:
        """Set operating mode for all motors. Disables torque first, does NOT re-enable."""
        with self._lock:
            for mid in self.motor_ids:
                self._write1(ADDR_TORQUE_ENABLE, mid, 0)
                self._write1(ADDR_OPERATING_MODE, mid, mode)

    def set_torque(self, enable: bool) -> None:
        """Enable or disable torque for all motors."""
        val = 1 if enable else 0
        with self._lock:
            for mid in self.motor_ids:
                self._write1(ADDR_TORQUE_ENABLE, mid, val)

    def read_positions(self) -> dict[int, int]:
        """SyncRead present position (4 bytes) from all motors. Returns signed raw values."""
        with self._lock:
            sync_read = GroupSyncRead(
                self._port_handler, self._packet_handler,
                ADDR_PRESENT_POSITION, 4,
            )
            for mid in self.motor_ids:
                sync_read.addParam(mid)

            result = sync_read.txRxPacket()
            if result != 0:
                raise RuntimeError(
                    f"SyncRead failed on {self.port}: "
                    f"{self._packet_handler.getTxRxResult(result)}"
                )

            positions = {}
            for mid in self.motor_ids:
                raw = sync_read.getData(mid, ADDR_PRESENT_POSITION, 4)
                positions[mid] = int(np.int32(np.uint32(raw)))

            sync_read.clearParam()
            return positions

    def write_positions(self, positions: dict[int, int]) -> None:
        """SyncWrite goal position (4 bytes) to specified motors."""
        with self._lock:
            sync_write = GroupSyncWrite(
                self._port_handler, self._packet_handler,
                ADDR_GOAL_POSITION, 4,
            )
            for mid, pos in positions.items():
                param = [
                    dxl_lobyte(dxl_loword(pos)),
                    dxl_hibyte(dxl_loword(pos)),
                    dxl_lobyte(dxl_hiword(pos)),
                    dxl_hibyte(dxl_hiword(pos)),
                ]
                sync_write.addParam(mid, param)

            result = sync_write.txPacket()
            if result != 0:
                raise RuntimeError(
                    f"SyncWrite positions failed on {self.port}: "
                    f"{self._packet_handler.getTxRxResult(result)}"
                )
            sync_write.clearParam()

    def write_currents(self, currents: dict[int, int]) -> None:
        """SyncWrite goal current (2 bytes) to specified motors."""
        with self._lock:
            sync_write = GroupSyncWrite(
                self._port_handler, self._packet_handler,
                ADDR_GOAL_CURRENT, 2,
            )
            for mid, cur in currents.items():
                param = [dxl_lobyte(cur), dxl_hibyte(cur)]
                sync_write.addParam(mid, param)

            result = sync_write.txPacket()
            if result != 0:
                raise RuntimeError(
                    f"SyncWrite currents failed on {self.port}: "
                    f"{self._packet_handler.getTxRxResult(result)}"
                )
            sync_write.clearParam()

    def close(self) -> None:
        """Disable torque and close the port."""
        try:
            self.set_torque(False)
        finally:
            self._port_handler.closePort()

    # --- Private helpers ---
    def _write1(self, addr: int, motor_id: int, value: int) -> None:
        """Write 1 byte. Must be called with lock held."""
        result, error = self._packet_handler.write1ByteTxRx(
            self._port_handler, motor_id, addr, value,
        )
        if result != 0:
            raise RuntimeError(
                f"Write1 failed (motor {motor_id}, addr {addr}): "
                f"{self._packet_handler.getTxRxResult(result)}"
            )
        if error != 0:
            raise RuntimeError(
                f"Write1 error (motor {motor_id}, addr {addr}): "
                f"{self._packet_handler.getRxPacketError(error)}"
            )
