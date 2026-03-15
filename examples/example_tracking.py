"""Example: Tracking mode — Franka leads, Joylo follows."""

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


def main():
    franka = MyFranka()
    joylo = Joylo(
        port_5v="/dev/franka_small",
        port_12v="/dev/franka_large",
        joint_signs=np.array([1, 1, 1, 1, 1, 1, 1], dtype=float),
        joint_offsets_rad=franka.read_joint_positions(),
    )
    system = JoyloSystem(franka, joylo, control_rate_hz=100.0)

    print("Starting tracking mode (Franka leads, Joylo follows)...")
    system.start_tracking()

    try:
        input("Press Enter to stop.\n")
    finally:
        system.close()
        print("Done.")


if __name__ == "__main__":
    main()
