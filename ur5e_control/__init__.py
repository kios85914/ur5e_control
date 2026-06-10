"""ur5e_control — pure control library for the UR5e over the URScript socket transport.

Public convenience imports (``UR5eRobot``, ``RobotConfig``, ``RobotState``) are
wired up in the final verification task, once all modules exist. During
development, import submodules directly, e.g. ``from ur5e_control.config import
RobotConfig``.
"""

__version__ = "0.1.0"
