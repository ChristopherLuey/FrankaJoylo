"""Example: Tracking with trigger — starts tracking, switches to teleop on trigger."""

import numpy as np

from franka_joylo import FrankaInterface, Joylo, JoyloSystem


# --- You must implement this for your Franka setup ---
class MyFranka(FrankaInterface):
    def read_joint_positions(self) -> np.ndarray:
        raise NotImplementedError("Implement for your Franka setup")

    def send_joint_positions(self, positions: np.ndarray) -> None:
        raise NotImplementedError("Implement for your Franka setup")

    def start_gravity_comp(self) -> None:
        raise NotImplementedError("Implement for your Franka setup")

    def stop_gravity_comp(self) -> None:
        raise NotImplementedError("Implement for your Franka setup")


_triggered = False


def trigger_fn() -> bool:
    """Replace with your own trigger logic (e.g., button press, force threshold)."""
    return _triggered


def main():
    franka = MyFranka()
    joylo = Joylo(
        port_5v="/dev/ttyUSB0",
        port_12v="/dev/ttyUSB1",
        joint_signs=np.array([1, 1, 1, 1, 1, 1, 1], dtype=float),
        joint_offsets_rad=franka.read_joint_positions(),
        gravity_comp_currents=np.array([0, 0, 0, 0, 0, 0, 0], dtype=int),
    )
    system = JoyloSystem(franka, joylo, control_rate_hz=100.0)

    print("Starting tracking mode with trigger...")
    print("When trigger_fn() returns True, system will switch to teleop.")
    system.start_tracking(trigger_fn=trigger_fn)

    try:
        input("Press Enter to stop.\n")
    finally:
        system.close()
        print("Done.")


if __name__ == "__main__":
    main()
