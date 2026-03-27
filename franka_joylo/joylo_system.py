"""Top-level orchestrator: tracking and teleop modes."""

import threading
import time
from collections.abc import Callable

import numpy as np

from franka_joylo.franka_interface import FrankaInterface
from franka_joylo.joylo import Joylo


class JoyloSystem:
    """Coordinates a Franka and a Joylo in tracking or teleop mode.

    Args:
        franka: A concrete FrankaInterface implementation.
        joylo: A configured Joylo instance.
        control_rate_hz: Target loop rate for control threads.
        smoothing_alpha: EMA smoothing factor for teleop (0 = all previous, 1 = all new).
    """

    def __init__(
        self,
        franka: FrankaInterface,
        joylo: Joylo,
        control_rate_hz: float = 100.0,
        smoothing_alpha: float = 0.95,
    ):
        self._franka = franka
        self._joylo = joylo
        self._dt = 1.0 / control_rate_hz
        self._alpha = smoothing_alpha

        self._stop_event = threading.Event()
        self._control_thread: threading.Thread | None = None

        self._franka_positions: np.ndarray | None = None
        self._joylo_positions: np.ndarray | None = None

    def get_franka_positions(self) -> np.ndarray | None:
        """Return the latest Franka joint positions (radians), or None if not yet read."""
        return self._franka_positions

    def get_joylo_positions(self) -> np.ndarray | None:
        """Return the latest Joylo joint positions (radians), or None if not yet read."""
        return self._joylo_positions

    def start_tracking(self, trigger_fn: Callable[[], bool] | None = None) -> None:
        """Start tracking mode: Franka leads, Joylo follows.

        Args:
            trigger_fn: Optional callable. When it returns True, auto-transition to teleop.
        """
        self._stop_control()
        self._joylo.set_position_mode()
        self._stop_event.clear()
        self._control_thread = threading.Thread(
            target=self._tracking_loop, args=(trigger_fn,), daemon=True,
        )
        self._control_thread.start()

    def start_teleop(self) -> None:
        """Start teleop mode: Joylo leads with gravity comp, Franka follows."""
        self._stop_control()
        self._joylo.disable_torque()
        self._joylo.set_current_mode()
        self._stop_event.clear()
        self._control_thread = threading.Thread(
            target=self._teleop_loop, daemon=True,
        )
        self._control_thread.start()

    def stop(self) -> None:
        """Stop the active control loop."""
        self._stop_control()

    def restart_teleop(self) -> None:
        """Restart the teleop loop without changing motor modes.

        Use when calibration parameters change mid-teleop and you need
        to reset the EMA smoother, but motors are already in current mode.
        """
        self._stop_control()
        self._stop_event.clear()
        self._control_thread = threading.Thread(
            target=self._teleop_loop, daemon=True,
        )
        self._control_thread.start()

    def close(self) -> None:
        """Stop control and clean up all resources."""
        self._stop_control()
        self._joylo.close()

    # --- Private control loops ---

    def _tracking_loop(self, trigger_fn: Callable[[], bool] | None) -> None:
        while not self._stop_event.is_set():
            t0 = time.monotonic()

            franka_q = self._franka.read_joint_positions()
            self._franka_positions = franka_q

            self._joylo.command_positions(franka_q)
            self._joylo_positions = self._joylo.joint_positions

            if trigger_fn is not None and trigger_fn():
                self._stop_event.set()
                self._transition_to_teleop()
                return

            elapsed = time.monotonic() - t0
            if elapsed < self._dt:
                time.sleep(self._dt - elapsed)

    def _teleop_loop(self) -> None:
        smoothed = self._franka.read_joint_positions()

        while not self._stop_event.is_set():
            t0 = time.monotonic()

            # Read Joylo first, command Franka ASAP, then do gravity comp.
            # Gravity comp is important but not latency-critical — moving it
            # after the Franka command keeps it off the read-to-command path.
            joylo_q = self._joylo.joint_positions
            self._joylo_positions = joylo_q

            smoothed = self._alpha * joylo_q + (1.0 - self._alpha) * smoothed

            self._franka.send_joint_positions(smoothed)
            self._franka_positions = self._franka.read_joint_positions()

            self._joylo.command_gravity_comp()

            elapsed = time.monotonic() - t0
            if elapsed < self._dt:
                time.sleep(self._dt - elapsed)

    def _transition_to_teleop(self) -> None:
        """Handle mode transition from tracking to teleop."""
        self._joylo.disable_torque()
        self._joylo.set_current_mode()
        self._stop_event.clear()
        self._control_thread = threading.Thread(
            target=self._teleop_loop, daemon=True,
        )
        self._control_thread.start()

    def _stop_control(self) -> None:
        """Signal the control thread to stop and wait for it."""
        self._stop_event.set()
        if self._control_thread is not None and self._control_thread.is_alive():
            self._control_thread.join(timeout=2.0)
        self._control_thread = None
