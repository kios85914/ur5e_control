"""ur5e_control — pure control library for the UR5e over the URScript socket transport.

Public, user-facing API::

    from ur5e_control import UR5eRobot, RobotConfig, RobotState

    with UR5eRobot(RobotConfig()) as robot:
        robot.move_l([0.1, 0.3, 0.2, 0.0, -3.14, 0.0])  # world frame, m/rad
        state = robot.get_state()
        robot.home()

Layers (low to high): :mod:`config`/:mod:`state` -> :mod:`safety`,
:mod:`connection`, :mod:`script_sender` -> :mod:`motion` -> :mod:`robot`. The
forward-looking force-control subsystem lives in :mod:`ur5e_control.force`
(pending Robotiq FT 300 hardware validation).

Units & frames: meters, radians, UR base frame. World<->UR conversion is explicit
in :class:`RobotConfig` (no hidden negation).
"""

from .config import RobotConfig
from .robot import UR5eRobot
from .state import RobotState, parse_state

__version__ = "0.1.0"

__all__ = ["UR5eRobot", "RobotConfig", "RobotState", "parse_state", "__version__"]
