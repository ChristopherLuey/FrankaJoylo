"""MuJoCo-based simulated Franka interface for testing without a real robot."""

from pathlib import Path

import mujoco
import numpy as np

from franka_joylo.franka_interface import FrankaInterface

MODEL_PATH = Path(__file__).resolve().parent.parent / "models" / "franka_panda.xml"
NUM_JOINTS = 7


class SimFrankaInterface(FrankaInterface):
    """Franka interface backed by a MuJoCo simulation.

    Uses position actuators to track commanded joint angles.
    Pass the same (model, data) to mujoco.viewer for visualization.
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data

    @classmethod
    def create(cls, model_path: str | Path | None = None) -> "SimFrankaInterface":
        """Load the Franka MJCF and return a SimFrankaInterface with its model/data."""
        path = str(model_path or MODEL_PATH)
        model = mujoco.MjModel.from_xml_path(path)
        data = mujoco.MjData(model)
        # Reset to home keyframe
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)
        return cls(model, data)

    def read_joint_positions(self) -> np.ndarray:
        return self.data.qpos[:NUM_JOINTS].copy()

    def send_joint_positions(self, positions: np.ndarray) -> None:
        self.data.ctrl[:NUM_JOINTS] = positions

    def start_gravity_comp(self) -> None:
        pass  # Position actuators continue tracking in sim

    def stop_gravity_comp(self) -> None:
        pass
