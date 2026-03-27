"""Low-level Dynamixel SDK wrapper — one instance per USB port."""

import os
import threading
import time

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

ADDR_HARDWARE_ERROR_STATUS = 70

PROTOCOL_VERSION = 2.0


class DxlDriver:
    """Manages a single Dynamixel bus (one USB port, multiple motors)."""

    def __init__(self, port: str, motor_ids: list[int], baudrate: int = 1000000):
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

        self._set_latency_timer(1)
        self._clear_hardware_errors()

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

    def read_positions(self, retries: int = 3) -> dict[int, int]:
        """SyncRead present position (4 bytes) from all motors. Returns signed raw values."""
        with self._lock:
            last_err = None
            for attempt in range(retries):
                sync_read = GroupSyncRead(
                    self._port_handler, self._packet_handler,
                    ADDR_PRESENT_POSITION, 4,
                )
                for mid in self.motor_ids:
                    sync_read.addParam(mid)

                result = sync_read.txRxPacket()
                if result != 0:
                    last_err = (
                        f"SyncRead failed on {self.port}: "
                        f"{self._packet_handler.getTxRxResult(result)}"
                    )
                    sync_read.clearParam()
                    time.sleep(0.001)
                    continue

                positions = {}
                for mid in self.motor_ids:
                    raw = sync_read.getData(mid, ADDR_PRESENT_POSITION, 4)
                    positions[mid] = int(np.int32(np.uint32(raw)))

                sync_read.clearParam()
                return positions

            raise RuntimeError(last_err)

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
        except RuntimeError as e:
            print(f"[DxlDriver] Warning: could not disable torque on {self.port}: {e}")
        finally:
            self._port_handler.closePort()

    # --- Private helpers ---
    def _set_latency_timer(self, ms: int) -> None:
        """Lower the FTDI USB-serial latency timer for faster reads.

        The Python Dynamixel SDK does not set this (the C version does).
        Default is 16 ms, which caps SyncRead throughput at ~60 Hz per bus.
        """
        real_path = os.path.realpath(self.port)
        dev_name = os.path.basename(real_path)
        sysfs_path = f"/sys/bus/usb-serial/devices/{dev_name}/latency_timer"
        try:
            with open(sysfs_path, "w") as f:
                f.write(str(ms))
        except PermissionError:
            print(f"[DxlDriver] Warning: cannot set latency timer on {self.port}. "
                  f"Run once: sudo chmod a+w {sysfs_path}")
        except FileNotFoundError:
            pass  # Not an FTDI adapter

    def _clear_hardware_errors(self) -> None:
        """Check each motor for hardware errors and reboot any that have them."""
        for mid in self.motor_ids:
            val, res, _ = self._packet_handler.read1ByteTxRx(
                self._port_handler, mid, ADDR_HARDWARE_ERROR_STATUS,
            )
            if res == 0 and val != 0:
                print(f"[DxlDriver] Motor {mid} on {self.port} has hardware error "
                      f"(status=0x{val:02x}), rebooting...")
                self._packet_handler.reboot(self._port_handler, mid)
                time.sleep(1.0)
                for attempt in range(10):
                    val2, res2, _ = self._packet_handler.read1ByteTxRx(
                        self._port_handler, mid, ADDR_HARDWARE_ERROR_STATUS,
                    )
                    if res2 == 0 and val2 == 0:
                        print(f"[DxlDriver] Motor {mid} on {self.port} recovered.")
                        break
                    time.sleep(0.5)
                else:
                    print(f"[DxlDriver] WARNING: Motor {mid} on {self.port} still has "
                          f"hardware error after reboot (status=0x{val2:02x}). "
                          f"Check power supply and wiring.")

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
