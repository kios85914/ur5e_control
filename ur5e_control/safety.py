"""Safety validators for UR5e motion commands.

Pure functions that enforce the limits declared on a :class:`RobotConfig`
before any command is sent to the robot. There is no socket logic here — these
helpers only inspect numbers and raise typed exceptions on violations so the
motion layer can fail fast and loudly.

Conventions (matching the rest of the library):

* Poses are 6-element lists ``[x, y, z, rx, ry, rz]`` in **meters** and
  **radians**, expressed in the **UR base frame**.
* Joint targets are 6-element lists ``[j0, j1, j2, j3, j4, j5]`` in
  **radians**.
* Speeds are scalars in **m/s** for Cartesian (moveL) motion or **rad/s** for
  joint (moveJ) motion; the safety cap is :attr:`RobotConfig.max_speed`.
"""

from __future__ import annotations

from typing import Sequence

from ur5e_control.config import RobotConfig

__all__ = [
    "SafetyViolation",
    "WorkspaceViolation",
    "SpeedViolation",
    "JointLimitViolation",
    "check_pose_in_workspace",
    "check_joints",
    "clamp_speed",
]

#: Axis order of a Cartesian pose's translational components.
_WORKSPACE_AXES = ("x", "y", "z")


class SafetyViolation(Exception):
    """Base class for all safety-limit violations.

    Catching this catches every typed violation below.
    """


class WorkspaceViolation(SafetyViolation):
    """Raised when a requested pose lies outside the configured workspace.

    Also raised when the pose is not a well-formed 6-element list.
    """


class SpeedViolation(SafetyViolation):
    """Raised when a requested speed is invalid (non-positive)."""


class JointLimitViolation(SafetyViolation):
    """Raised when joint targets are the wrong count or out of range."""


def check_pose_in_workspace(pose: Sequence[float], config: RobotConfig) -> bool:
    """Validate that a Cartesian pose lies within the configured workspace.

    Only the translational components ``x, y, z`` (meters, UR base frame) are
    bounds-checked against :attr:`RobotConfig.workspace_limits`; the rotation
    components ``rx, ry, rz`` (radians) are not constrained here. Bounds are
    inclusive.

    Args:
        pose: 6-element sequence ``[x, y, z, rx, ry, rz]`` in meters/radians,
            UR base frame.
        config: Robot configuration supplying ``workspace_limits``.

    Returns:
        ``True`` if the pose is within bounds.

    Raises:
        WorkspaceViolation: If ``pose`` is not length 6, or any of x/y/z falls
            outside its ``(min, max)`` limit. The message names the offending
            axis.
    """
    if len(pose) != 6:
        raise WorkspaceViolation(
            f"pose must have 6 elements [x, y, z, rx, ry, rz], got {len(pose)}"
        )

    for axis, value in zip(_WORKSPACE_AXES, pose[:3]):
        lo, hi = config.workspace_limits[axis]
        if value < lo or value > hi:
            raise WorkspaceViolation(
                f"{axis}={value:.4f} m is outside workspace limit "
                f"[{lo:.4f}, {hi:.4f}] m"
            )
    return True


def check_joints(joints: Sequence[float], config: RobotConfig) -> bool:
    """Validate that joint targets are 6 angles within their limits.

    Args:
        joints: 6-element sequence ``[j0, j1, j2, j3, j4, j5]`` in radians.
        config: Robot configuration supplying ``joint_limits`` (a 6-tuple of
            ``(min, max)`` pairs in radians).

    Returns:
        ``True`` if all six joints are within bounds.

    Raises:
        JointLimitViolation: If ``joints`` is not length 6, or any joint falls
            outside its ``(min, max)`` limit. The message names the offending
            joint index. Bounds are inclusive.
    """
    if len(joints) != 6:
        raise JointLimitViolation(
            f"joints must have 6 elements [j0..j5], got {len(joints)}"
        )

    for index, (value, (lo, hi)) in enumerate(zip(joints, config.joint_limits)):
        if value < lo or value > hi:
            raise JointLimitViolation(
                f"joint {index}={value:.4f} rad is outside limit "
                f"[{lo:.4f}, {hi:.4f}] rad"
            )
    return True


def clamp_speed(speed: float, config: RobotConfig) -> float:
    """Clamp a requested speed to the configured safety cap.

    Speeds above :attr:`RobotConfig.max_speed` are reduced to ``max_speed``;
    speeds at or below the cap are returned unchanged. A non-positive speed is
    rejected (it would never converge), as a typed error rather than a silent
    clamp.

    Args:
        speed: Requested speed in m/s (moveL) or rad/s (moveJ). Must be > 0.
        config: Robot configuration supplying ``max_speed`` (m/s).

    Returns:
        The speed, capped at ``config.max_speed``.

    Raises:
        SpeedViolation: If ``speed`` is not strictly positive.
    """
    if speed <= 0.0:
        raise SpeedViolation(f"speed must be positive, got {speed}")
    return min(speed, config.max_speed)
