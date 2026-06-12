"""Configuration and frame transforms for the UR5e control library.

This module defines :class:`RobotConfig`, the single source of truth for
network endpoints, motion defaults, safety limits, and the world<->UR-base
frame conversion. There is no hidden negation anywhere else in the library:
all world<->UR coordinate conversion goes through :meth:`RobotConfig.world_to_ur`
and :meth:`RobotConfig.ur_to_world`.

Units and frames (used throughout the whole library):

* Positions are in **meters**, rotations in **radians**.
* A *pose* is the 6-element list ``[x, y, z, rx, ry, rz]`` where ``rx, ry, rz``
  is the axis-angle (rotation-vector) orientation.
* *Joints* are the 6-element list ``[j0, j1, j2, j3, j4, j5]`` in radians.
* Geometry stored in this config (``home_pose``, ``workspace_limits``,
  ``joint_limits``) and exchanged with the daemon is expressed in the **UR base
  frame**. The world frame differs only by negating the x and y axes.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple


def _default_workspace_limits() -> Dict[str, Tuple[float, float]]:
    """Default Cartesian workspace clamps in the UR base frame (meters).

    Returns a fresh dict mapping each axis to its ``(min, max)`` bound so that
    every :class:`RobotConfig` instance owns an independent copy.
    """
    return {
        "x": (-0.40, 0.40),
        "y": (-0.565, -0.265),
        "z": (-0.10, 0.40),
    }


def _default_joint_limits() -> Tuple[Tuple[float, float], ...]:
    """Default per-joint limits in radians: six ``(min, max)`` pairs.

    Tuples are immutable, so a single shared default is safe to reuse.
    """
    return tuple((-6.283185, 6.283185) for _ in range(6))


def _default_home_pose() -> List[float]:
    """Default home pose ``[x, y, z, rx, ry, rz]`` (meters, radians) in UR base frame.

    Returns a fresh list so each :class:`RobotConfig` instance owns its own copy.
    """
    return [0.0, -0.35, 0.25, 0.0, -3.14, 0.0]


@dataclass
class RobotConfig:
    """Network, motion, and safety configuration for a UR5e robot.

    All distances are in meters and all rotations in radians. Geometry fields
    (``home_pose``, ``workspace_limits``, ``joint_limits``) are expressed in the
    UR base frame; convert to/from the world frame with :meth:`world_to_ur` and
    :meth:`ur_to_world`.

    Attributes:
        controller_ip: IP address of the UR controller (the daemon host).
        script_port: TCP port the controller listens on for URScript.
        pc_host: IP address of this PC, advertised to the daemon for the state
            stream callback.
        state_port: TCP port on this PC that receives the daemon state stream.
        default_speed: Default Cartesian/joint speed (m/s or rad/s).
        default_accel: Default Cartesian/joint acceleration (m/s^2 or rad/s^2).
        default_move_time: Default move duration in seconds for moveL/moveJ.
            **Leave this at 0.0 to make ``speed`` actually take effect.** In
            URScript, ``movel``/``movej`` accept ``(pose, a, v, t)``; when ``t``
            (time) is non-zero it OVERRIDES ``v`` (speed) and ``a`` (accel) and
            the move always takes exactly ``t`` seconds. So a non-zero
            ``default_move_time`` makes every move ignore the ``speed`` argument
            (the classic "changing speed does nothing" symptom). With ``0.0`` the
            controller honours ``v``/``a`` and ``speed`` works as expected.
        convergence_tol: Tolerance (meters/radians) for deciding a blocking move
            has reached its target.
        workspace_limits: Cartesian clamps in the UR base frame, mapping each of
            ``"x"``, ``"y"``, ``"z"`` to a ``(min, max)`` tuple in meters.
        joint_limits: Six ``(min, max)`` tuples in radians, one per joint.
        max_speed: Hard upper bound on commanded speed (m/s or rad/s).
        home_pose: Home pose ``[x, y, z, rx, ry, rz]`` (meters, radians) in the
            UR base frame.
        force_mode_enabled: Whether the daemon's ``force_mode`` branches (cmd 4
            maintain-force and cmd 7 impedance) are armed. Injected into the
            uploaded daemon by :func:`script_sender.render_daemon` as
            ``FORCE_MODE_ENABLED``. **Kept ``False`` by default** so a stray force
            command cannot make the arm compliant / drive it into a surface before
            the FT 300 is mounted and parameters are validated. Set ``True`` only
            after on-robot bring-up. (Guarded move, cmd 5, is a plain velocity
            move and is NOT gated by this.)
    """

    controller_ip: str = "192.168.0.137"
    script_port: int = 30001
    pc_host: str = "192.168.0.120"
    state_port: int = 30002
    default_speed: float = 0.1
    default_accel: float = 0.1
    default_move_time: float = 0.0   # 0 => speed/accel govern (see attr docstring)
    convergence_tol: float = 1e-3
    workspace_limits: Dict[str, Tuple[float, float]] = field(
        default_factory=_default_workspace_limits
    )
    joint_limits: Tuple[Tuple[float, float], ...] = field(
        default_factory=_default_joint_limits
    )
    max_speed: float = 0.25
    home_pose: List[float] = field(default_factory=_default_home_pose)
    force_mode_enabled: bool = False

    def world_to_ur(self, pose: List[float]) -> List[float]:
        """Convert a pose from the world frame to the UR base frame.

        Negates only the x and y position axes; z and the rotation vector
        ``(rx, ry, rz)`` are passed through unchanged. The input list is not
        mutated.

        Args:
            pose: World-frame pose ``[x, y, z, rx, ry, rz]`` in meters/radians.

        Returns:
            A new UR-base-frame pose ``[x, y, z, rx, ry, rz]`` in meters/radians.
        """
        x, y, z, rx, ry, rz = pose
        return [x, y, z, rx, ry, rz]

    def ur_to_world(self, pose: List[float]) -> List[float]:
        """Convert a pose from the UR base frame to the world frame.

        Inverse of :meth:`world_to_ur`: negates only the x and y position axes;
        z and the rotation vector are unchanged. The input list is not mutated.
        Together with :meth:`world_to_ur` this round-trips to the identity.

        Args:
            pose: UR-base-frame pose ``[x, y, z, rx, ry, rz]`` in meters/radians.

        Returns:
            A new world-frame pose ``[x, y, z, rx, ry, rz]`` in meters/radians.
        """
        x, y, z, rx, ry, rz = pose
        return [x, y, z, rx, ry, rz]
