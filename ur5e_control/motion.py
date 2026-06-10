"""Motion command encoding and execution for the UR5e control library.

:class:`MotionController` is the layer that turns high-level motion requests
(move to a Cartesian pose, move to a joint configuration, stop, go home) into the
fixed-format command tuples understood by the URScript motion daemon, applies the
world<->UR frame transform and safety checks, pushes the encoded string through a
:class:`~ur5e_control.connection.RobotConnection`, and (optionally) blocks until
the robot has converged on the commanded target.

Units and frames (matching the rest of the library):

* Positions are in **meters**, rotations/joints in **radians**.
* A *pose* is ``[x, y, z, rx, ry, rz]`` (axis-angle orientation); *joints* are
  ``[j0, j1, j2, j3, j4, j5]``.
* Cartesian inputs to :meth:`move_l` are in the **world frame**; they are
  converted to the **UR base frame** via :meth:`RobotConfig.world_to_ur` before
  any safety check or encoding. Joint inputs to :meth:`move_j` are not
  frame-dependent and are used as-is.

Command protocol (PC -> daemon), the fixed 10-float ASCII tuple this module
emits (read by the daemon with ``socket_read_ascii_float(10, ...)``)::

    "(cmd, a0, a1, a2, a3, a4, a5, accel, vel, time)"

* ``cmd = 0`` moveL : ``a0..a5`` = pose ``x, y, z, rx, ry, rz`` (m, rad);
  ``accel`` m/s^2, ``vel`` m/s, ``time`` s.
* ``cmd = 1`` moveJ : ``a0..a5`` = joints ``j0..j5`` (rad);
  ``accel`` rad/s^2, ``vel`` rad/s, ``time`` s.
* ``cmd = 2`` stop  : ``a0`` = deceleration m/s^2; ``a1..a5`` = 0.
* ``cmd = 3`` home  : payload all 0 (the daemon uses its configured home pose).

Blocking moves poll :meth:`RobotConnection.latest_state`, parse it with
:func:`ur5e_control.state.parse_state`, and exit once every TCP-pose component is
within :attr:`RobotConfig.convergence_tol` of the (UR-base-frame) target. This
``convergence_tol`` comparison replaces the legacy hard-coded ``0.0001``
threshold.
"""

from __future__ import annotations

import time
from typing import List, Optional, Sequence

from .config import RobotConfig
from .connection import RobotConnection
from .safety import check_joints, check_pose_in_workspace, clamp_speed
from .state import parse_state

__all__ = ["MotionController"]

# Command opcodes for the PC -> daemon tuple (see module docstring / protocol).
_CMD_MOVEL = 0
_CMD_MOVEJ = 1
_CMD_STOP = 2
_CMD_HOME = 3

# Number of scalar values in a pose / joint payload.
_PAYLOAD_LEN = 6

# Default deceleration (m/s^2) used by :meth:`stop` when none is supplied.
_DEFAULT_STOP_DECEL = 2.0

# Seconds to sleep between state polls in a blocking move (poll period).
_POLL_INTERVAL_S = 0.05

# Safety ceiling on the number of poll iterations in a blocking move, so a
# never-converging stream cannot wedge the caller forever.
_MAX_POLL_ITERS = 2000


class MotionController:
    """Encode, validate, send, and (optionally) await UR5e motion commands.

    The controller is transport-agnostic above the socket: it depends only on a
    ``connection`` exposing ``send(str)`` and ``latest_state() -> str`` (the real
    :class:`~ur5e_control.connection.RobotConnection`, or any compatible
    stand-in/mock). All geometry handling — world<->UR conversion, workspace and
    joint-limit checks, speed clamping — happens here before a byte is sent.

    Args:
        connection: Transport providing ``send(msg: str) -> None`` and
            ``latest_state() -> str``. Strings produced by this controller are
            sent verbatim.
        config: Robot configuration supplying frame transforms, motion defaults
            (speed/accel/move time), safety limits, and ``convergence_tol``.
            Defaults to a fresh :class:`RobotConfig`.
    """

    def __init__(
        self,
        connection: RobotConnection,
        config: RobotConfig = RobotConfig(),
    ) -> None:
        self._conn = connection
        self._config = config

    # ------------------------------------------------------------------
    # Encoding
    # ------------------------------------------------------------------
    @staticmethod
    def encode_command(
        cmd: int,
        payload: Sequence[float],
        accel: float,
        vel: float,
        time: float,
    ) -> str:
        """Encode one command into the fixed 10-field ASCII tuple string.

        Produces exactly ``"(cmd, a0, a1, a2, a3, a4, a5, accel, vel, time)"``
        with comma-space separators. ``cmd`` is rendered as an integer opcode;
        the six payload values and the three motion parameters are rendered with
        Python's shortest round-trip float representation (e.g. ``0.1``, ``0.0``,
        ``-3.14``). The daemon reads this back with
        ``socket_read_ascii_float(10, ...)``.

        The meaning of ``payload`` depends on ``cmd`` (see the module docstring):
        a Cartesian pose ``[x, y, z, rx, ry, rz]`` in meters/radians for moveL, a
        joint vector ``[j0..j5]`` in radians for moveJ, deceleration-in-``a0`` for
        stop, or all zeros for home. ``accel``/``vel`` are in m/s^2,m/s (moveL) or
        rad/s^2,rad/s (moveJ); ``time`` is in seconds.

        Args:
            cmd: Command opcode (0 moveL, 1 moveJ, 2 stop, 3 home, 4 force).
            payload: Exactly six floats (``a0..a5``); meaning depends on ``cmd``.
            accel: Acceleration (m/s^2 or rad/s^2).
            vel: Speed (m/s or rad/s).
            time: Move/blend duration in seconds.

        Returns:
            The exact ``"(...)"`` tuple string ready to hand to ``connection.send``.

        Raises:
            ValueError: If ``payload`` does not contain exactly six values.
        """
        payload = list(payload)
        if len(payload) != _PAYLOAD_LEN:
            raise ValueError(
                f"payload must have {_PAYLOAD_LEN} floats (a0..a5), got {len(payload)}"
            )

        fields = [str(int(cmd))]
        fields.extend(str(float(v)) for v in payload)
        fields.append(str(float(accel)))
        fields.append(str(float(vel)))
        fields.append(str(float(time)))
        return "(" + ", ".join(fields) + ")"

    # ------------------------------------------------------------------
    # Cartesian motion (moveL)
    # ------------------------------------------------------------------
    def move_l(
        self,
        pose: Sequence[float],
        speed: Optional[float] = None,
        accel: Optional[float] = None,
        blocking: bool = True,
        relative: bool = False,
    ) -> None:
        """Move the TCP linearly to a Cartesian pose (cmd=0 moveL).

        The input ``pose`` is in the **world frame**, meters/radians,
        ``[x, y, z, rx, ry, rz]``. When ``relative`` is ``True`` it is treated as
        a world-frame delta added to the current TCP pose (obtained from the
        latest daemon state and converted to world frame). The resulting absolute
        world pose is converted to the UR base frame via
        :meth:`RobotConfig.world_to_ur`, workspace-checked, encoded, and sent.

        Args:
            pose: World-frame target (or delta if ``relative``)
                ``[x, y, z, rx, ry, rz]`` in meters/radians.
            speed: Cartesian speed in m/s. ``None`` uses ``config.default_speed``.
                Clamped to ``config.max_speed``; must be > 0.
            accel: Cartesian acceleration in m/s^2. ``None`` uses
                ``config.default_accel``.
            blocking: If ``True``, poll state until the TCP pose is within
                ``config.convergence_tol`` of the (UR-frame) target.
            relative: If ``True``, ``pose`` is a delta on the current world pose.

        Raises:
            WorkspaceViolation: If the target pose is malformed or outside the
                configured workspace (nothing is sent).
            SpeedViolation: If ``speed`` is not strictly positive (nothing is
                sent).
        """
        target_world = list(pose)
        if relative:
            current_world = self._current_world_pose()
            target_world = [current_world[i] + target_world[i] for i in range(_PAYLOAD_LEN)]

        speed = self._config.default_speed if speed is None else speed
        accel = self._config.default_accel if accel is None else accel

        # Clamp/validate speed first (fail fast on a non-positive speed).
        vel = clamp_speed(speed, self._config)

        # World -> UR base frame, then workspace safety on the UR-frame pose.
        ur_pose = self._config.world_to_ur(target_world)
        check_pose_in_workspace(ur_pose, self._config)

        msg = self.encode_command(
            _CMD_MOVEL, ur_pose, accel, vel, self._config.default_move_time
        )
        self._conn.send(msg)

        if blocking:
            self._await_convergence(ur_pose)

    # ------------------------------------------------------------------
    # Joint motion (moveJ)
    # ------------------------------------------------------------------
    def move_j(
        self,
        joints: Sequence[float],
        speed: Optional[float] = None,
        accel: Optional[float] = None,
        blocking: bool = True,
    ) -> None:
        """Move to a joint configuration (cmd=1 moveJ).

        ``joints`` is ``[j0, j1, j2, j3, j4, j5]`` in radians. Joints are not
        frame-dependent, so no world<->UR conversion is applied. The targets are
        joint-limit checked, encoded, and sent.

        Args:
            joints: Target joint angles ``[j0..j5]`` in radians.
            speed: Joint speed in rad/s. ``None`` uses ``config.default_speed``.
                Clamped to ``config.max_speed``; must be > 0.
            accel: Joint acceleration in rad/s^2. ``None`` uses
                ``config.default_accel``.
            blocking: If ``True``, poll state until the TCP pose has settled (the
                joint move has effectively stopped) within
                ``config.convergence_tol``.

        Raises:
            JointLimitViolation: If ``joints`` is malformed or any joint is out of
                range (nothing is sent).
            SpeedViolation: If ``speed`` is not strictly positive (nothing is
                sent).
        """
        joints = list(joints)
        speed = self._config.default_speed if speed is None else speed
        accel = self._config.default_accel if accel is None else accel

        vel = clamp_speed(speed, self._config)
        check_joints(joints, self._config)

        msg = self.encode_command(
            _CMD_MOVEJ, joints, accel, vel, self._config.default_move_time
        )
        self._conn.send(msg)

        if blocking:
            # After a joint move we don't know the target TCP pose a priori, so
            # wait for the TCP pose to settle (successive states stop changing).
            self._await_settle()

    # ------------------------------------------------------------------
    # Stop / home
    # ------------------------------------------------------------------
    def stop(self, deceleration: float = _DEFAULT_STOP_DECEL) -> None:
        """Command an immediate controlled stop (cmd=2).

        Sends a stop tuple carrying the deceleration in ``a0`` (m/s^2) and zeros
        for ``a1..a5``; accel/vel/time fields are zero (unused by the daemon for
        a stop). Non-blocking — there is no target to converge on.

        Args:
            deceleration: Stop deceleration in m/s^2 (must be > 0 to be useful).
        """
        payload = [float(deceleration), 0.0, 0.0, 0.0, 0.0, 0.0]
        msg = self.encode_command(_CMD_STOP, payload, 0.0, 0.0, 0.0)
        self._conn.send(msg)

    def home(
        self,
        speed: Optional[float] = None,
        accel: Optional[float] = None,
        blocking: bool = True,
    ) -> None:
        """Move to the configured home pose (cmd=3).

        Sends a home tuple with an all-zero payload; the daemon substitutes its
        configured home pose (``config.home_pose``, UR base frame). When
        ``blocking`` is ``True``, polls state until the TCP pose is within
        ``config.convergence_tol`` of ``config.home_pose``.

        Args:
            speed: Speed in m/s. ``None`` uses ``config.default_speed``. Clamped
                to ``config.max_speed``; must be > 0.
            accel: Acceleration in m/s^2. ``None`` uses ``config.default_accel``.
            blocking: If ``True``, poll until converged on ``config.home_pose``.

        Raises:
            SpeedViolation: If ``speed`` is not strictly positive (nothing is
                sent).
        """
        speed = self._config.default_speed if speed is None else speed
        accel = self._config.default_accel if accel is None else accel
        vel = clamp_speed(speed, self._config)

        payload = [0.0] * _PAYLOAD_LEN
        msg = self.encode_command(
            _CMD_HOME, payload, accel, vel, self._config.default_move_time
        )
        self._conn.send(msg)

        if blocking:
            # The daemon homes to config.home_pose (already UR base frame).
            self._await_convergence(list(self._config.home_pose))

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _current_world_pose(self) -> List[float]:
        """Return the current TCP pose in the **world frame** (meters/radians).

        Reads the latest daemon state, parses it, and converts the UR-base-frame
        TCP pose to world frame via :meth:`RobotConfig.ur_to_world`.

        Raises:
            ValueError: If no valid state frame is available yet (a relative move
                needs a known current pose).
        """
        raw = self._conn.latest_state()
        if not raw:
            raise ValueError(
                "relative move requires a current pose, but no state is available yet"
            )
        state = parse_state(raw)
        return self._config.ur_to_world(state.tcp_pose)

    def _await_convergence(self, ur_target: Sequence[float]) -> None:
        """Block until the TCP pose is within ``convergence_tol`` of ``ur_target``.

        Polls :meth:`RobotConnection.latest_state` every ``_POLL_INTERVAL_S``
        seconds, parses each frame with :func:`parse_state`, and returns when
        ``max(abs(tcp_pose[i] - ur_target[i]))`` across all six components is
        below :attr:`RobotConfig.convergence_tol`. Empty or malformed frames are
        skipped (the robot may not have reported yet). This replaces the legacy
        magic ``0.0001`` threshold with the configurable tolerance.

        Args:
            ur_target: Target TCP pose ``[x, y, z, rx, ry, rz]`` in the UR base
                frame (meters/radians) to converge on.

        Raises:
            TimeoutError: If convergence is not reached within ``_MAX_POLL_ITERS``
                poll iterations (guards against a never-converging stream).
        """
        tol = self._config.convergence_tol
        for _ in range(_MAX_POLL_ITERS):
            raw = self._conn.latest_state()
            if raw:
                try:
                    state = parse_state(raw)
                except ValueError:
                    state = None
                if state is not None:
                    delta = max(
                        abs(state.tcp_pose[i] - ur_target[i])
                        for i in range(_PAYLOAD_LEN)
                    )
                    if delta < tol:
                        return
            time.sleep(_POLL_INTERVAL_S)
        raise TimeoutError(
            f"move did not converge within {_MAX_POLL_ITERS} polls "
            f"(tol={tol} m/rad)"
        )

    def _await_settle(self) -> None:
        """Block until the TCP pose stops changing (within ``convergence_tol``).

        Used for joint moves, where the resulting TCP pose is not known ahead of
        time. Polls successive state frames and returns once two consecutive
        parsed poses differ by less than :attr:`RobotConfig.convergence_tol` on
        every component (the motion has effectively settled).

        Raises:
            TimeoutError: If the pose does not settle within ``_MAX_POLL_ITERS``
                poll iterations.
        """
        tol = self._config.convergence_tol
        prev: Optional[List[float]] = None
        for _ in range(_MAX_POLL_ITERS):
            raw = self._conn.latest_state()
            if raw:
                try:
                    state = parse_state(raw)
                except ValueError:
                    state = None
                if state is not None:
                    if prev is not None:
                        delta = max(
                            abs(state.tcp_pose[i] - prev[i])
                            for i in range(_PAYLOAD_LEN)
                        )
                        if delta < tol:
                            return
                    prev = list(state.tcp_pose)
            time.sleep(_POLL_INTERVAL_S)
        raise TimeoutError(
            f"joint move did not settle within {_MAX_POLL_ITERS} polls "
            f"(tol={tol} rad)"
        )
