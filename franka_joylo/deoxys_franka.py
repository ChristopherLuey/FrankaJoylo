"""FrankaInterface adapter for the deoxys robot driver.

Requires the ``deoxys`` package (only available on the robot workstation).
"""

import copy

import numpy as np

from franka_joylo.franka_interface import FrankaInterface

# Default impedance gains — stiff for normal position tracking.
DEFAULT_STIFF_KP = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
DEFAULT_STIFF_KD = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]

# Compliant gains for gravity-comp-like mode — robot is soft but won't collapse.
DEFAULT_COMPLIANT_KP = [10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 2.0]
DEFAULT_COMPLIANT_KD = [6.0, 6.0, 6.0, 6.0, 4.0, 4.0, 2.0]


class DeoxysFrankaInterface(FrankaInterface):
    """Wraps the deoxys ``FrankaInterface`` to satisfy :class:`franka_joylo.FrankaInterface`.

    Args:
        config_path: Path to the deoxys YAML config.  Defaults to
            ``<deoxys.config_root>/charmander.yml``.
        control_freq: Control frequency in Hz sent to the deoxys driver.
        gripper_action: Constant gripper action appended to every command
            (-1 = open, 1 = close).
        stiff_kp: Joint stiffness gains for normal position tracking.
        stiff_kd: Joint damping gains for normal position tracking.
        compliant_kp: Joint stiffness gains for compliant (gravity-comp) mode.
        compliant_kd: Joint damping gains for compliant (gravity-comp) mode.
    """

    def __init__(
        self,
        config_path: str | None = None,
        control_freq: int = 100,
        gripper_action: float = -1.0,
        stiff_kp: list[float] | None = None,
        stiff_kd: list[float] | None = None,
        compliant_kp: list[float] | None = None,
        compliant_kd: list[float] | None = None,
    ):
        from deoxys import config_root
        from deoxys.franka_interface import FrankaInterface as DeoxysRobotInterface
        from deoxys.utils.config_utils import get_default_controller_config

        if config_path is None:
            config_path = config_root + "/charmander.yml"

        self._robot = DeoxysRobotInterface(
            config_path,
            use_visualizer=False,
            has_gripper=False,
            control_freq=control_freq,
            automatic_gripper_reset=False,
        )

        self._gripper_action = gripper_action

        # Build the base controller config from deoxys defaults, then override gains.
        self._base_cfg = get_default_controller_config("JOINT_IMPEDANCE")

        self._stiff_kp = list(stiff_kp or DEFAULT_STIFF_KP)
        self._stiff_kd = list(stiff_kd or DEFAULT_STIFF_KD)
        self._compliant_kp = list(compliant_kp or DEFAULT_COMPLIANT_KP)
        self._compliant_kd = list(compliant_kd or DEFAULT_COMPLIANT_KD)

        # Start in stiff mode.
        self._active_kp = self._stiff_kp
        self._active_kd = self._stiff_kd

    # --- FrankaInterface implementation ---

    def read_joint_positions(self) -> np.ndarray:
        return self._robot.last_q.copy()

    def send_joint_positions(self, positions: np.ndarray) -> None:
        action = np.concatenate([positions, [self._gripper_action]])
        cfg = copy.deepcopy(self._base_cfg)
        cfg["joint_kp"] = list(self._active_kp)
        cfg["joint_kd"] = list(self._active_kd)
        self._robot.control(
            controller_type="JOINT_IMPEDANCE",
            action=action,
            controller_cfg=cfg,
        )

    def start_gravity_comp(self) -> None:
        self._active_kp = self._compliant_kp
        self._active_kd = self._compliant_kd
        # Send current position with new gains so the switch is jerk-free.
        self.send_joint_positions(self.read_joint_positions())

    def stop_gravity_comp(self) -> None:
        self._active_kp = self._stiff_kp
        self._active_kd = self._stiff_kd
        self.send_joint_positions(self.read_joint_positions())

    # --- Extra helpers ---

    @property
    def robot_interface(self):
        """Direct access to the underlying deoxys robot interface."""
        return self._robot

    def close(self) -> None:
        self._robot.close()
