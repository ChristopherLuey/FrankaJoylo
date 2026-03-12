"""Abstract base class for Franka communication."""

from abc import ABC, abstractmethod

import numpy as np


class FrankaInterface(ABC):
    """User must subclass this to provide their own Franka communication layer."""

    @abstractmethod
    def read_joint_positions(self) -> np.ndarray:
        """Return current joint positions as shape (7,) array in radians."""
        ...

    @abstractmethod
    def send_joint_positions(self, positions: np.ndarray) -> None:
        """Command 7-DOF joint positions in radians."""
        ...

    @abstractmethod
    def start_gravity_comp(self) -> None:
        """Put the Franka in compliant (gravity compensation) mode."""
        ...

    @abstractmethod
    def stop_gravity_comp(self) -> None:
        """Return the Franka to position control mode."""
        ...
