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

import math
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
_CMD_FORCE = 4        # maintain force (persistent force_mode)
_CMD_SPEEDL = 5       # Cartesian velocity (guarded move)
_CMD_END_FORCE = 6    # exit any force mode and hold
_CMD_IMPEDANCE = 7    # compliant spring about the entry pose (hold here)
_CMD_IMPEDANCE_MOVE = 8  # compliant spring whose equilibrium is a target pose

# Default watchdog (s) for speedl: the robot stops on its own if no new command
# arrives within this window (hardware dead-man for the guarded move).
_DEFAULT_SPEEDL_WATCHDOG_S = 1.0

# Number of scalar values in a pose / joint payload.
_PAYLOAD_LEN = 6

# Length of a Cartesian position / direction triplet.
_VEC3_LEN = 3

# Default deceleration (m/s^2) used by :meth:`stop` when none is supplied.
_DEFAULT_STOP_DECEL = 2.0

# Seconds to sleep between state polls in a blocking move (poll period).
_POLL_INTERVAL_S = 0.05

# Safety ceiling on the number of poll iterations in a blocking move, so a
# never-converging stream cannot wedge the caller forever.
_MAX_POLL_ITERS = 2000


def _rotvec_to_quat(rx: float, ry: float, rz: float) -> tuple:
    """Convert an axis-angle (rotation-vector) orientation to a unit quaternion.

    UR poses carry orientation as a rotation vector ``[rx, ry, rz]`` (axis * angle
    in radians). Returns ``(w, x, y, z)``.
    """
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    s = math.sin(angle / 2.0)
    return (math.cos(angle / 2.0), rx / angle * s, ry / angle * s, rz / angle * s)


def _orientation_angle(a: Sequence[float], b: Sequence[float]) -> float:
    """Smallest rotation angle (rad) between two orientations given as rotvecs.

    Comparing rotation-vector COMPONENTS directly is wrong near 180 degrees: the
    same physical orientation has two equivalent representations (the axis-angle
    "double cover"), e.g. ``[0,-3.14,0.5]`` and its wrapped/negated form describe
    the same pose but differ wildly component-by-component. This converts both to
    quaternions and returns the true angular distance (``abs`` of the dot handles
    the double cover), so equivalent representations register as ~0.

    Args:
        a, b: rotation vectors ``[rx, ry, rz]`` (radians).

    Returns:
        The angle between the two orientations in radians, in ``[0, pi]``.
    """
    qa = _rotvec_to_quat(a[0], a[1], a[2])
    qb = _rotvec_to_quat(b[0], b[1], b[2])
    dot = abs(sum(x * y for x, y in zip(qa, qb)))
    return 2.0 * math.acos(min(1.0, dot))


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
        move_time: Optional[float] = None,
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
            move_time: Move duration in seconds (URScript ``t``). ``None`` uses
                ``config.default_move_time``. **If > 0 it OVERRIDES ``speed`` and
                ``accel``** — the move takes exactly ``move_time`` seconds (this is
                URScript ``movel`` behavior). Use ``0.0`` (the default) to let
                ``speed``/``accel`` govern. Must be >= 0.

        Raises:
            WorkspaceViolation: If the target pose is malformed or outside the
                configured workspace (nothing is sent).
            SpeedViolation: If ``speed`` is not strictly positive (nothing is
                sent).
            ValueError: If ``move_time`` is negative.
        """
        target_world = list(pose)
        if relative:
            current_world = self._current_world_pose()
            target_world = [current_world[i] + target_world[i] for i in range(_PAYLOAD_LEN)]

        speed = self._config.default_speed if speed is None else speed
        accel = self._config.default_accel if accel is None else accel
        t = self._resolve_move_time(move_time)

        # Clamp/validate speed first (fail fast on a non-positive speed).
        vel = clamp_speed(speed, self._config)

        # World -> UR base frame, then workspace safety on the UR-frame pose.
        ur_pose = self._config.world_to_ur(target_world)
        check_pose_in_workspace(ur_pose, self._config)

        msg = self.encode_command(_CMD_MOVEL, ur_pose, accel, vel, t)
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
        move_time: Optional[float] = None,
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
            blocking: If ``True``, poll state until every joint is within
                ``config.convergence_tol`` of the commanded target.
            move_time: Move duration in seconds (URScript ``t``). ``None`` uses
                ``config.default_move_time``. **If > 0 it OVERRIDES ``speed`` and
                ``accel``** — the move takes exactly ``move_time`` seconds
                (URScript ``movej`` behavior). ``0.0`` lets ``speed``/``accel``
                govern. Must be >= 0.

        Raises:
            JointLimitViolation: If ``joints`` is malformed or any joint is out of
                range (nothing is sent).
            SpeedViolation: If ``speed`` is not strictly positive (nothing is
                sent).
            ValueError: If ``move_time`` is negative.
        """
        joints = list(joints)
        speed = self._config.default_speed if speed is None else speed
        accel = self._config.default_accel if accel is None else accel
        t = self._resolve_move_time(move_time)

        vel = clamp_speed(speed, self._config)
        check_joints(joints, self._config)

        msg = self.encode_command(_CMD_MOVEJ, joints, accel, vel, t)
        self._conn.send(msg)

        if blocking:
            # Wait until the joints actually reach the commanded target (the
            # state stream carries joint angles). This blocks for the whole move,
            # so the next command can't pre-empt an unfinished joint move.
            self._await_joint_convergence(joints)

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
        move_time: Optional[float] = None,
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
            move_time: Move duration in seconds (URScript ``t``). ``None`` uses
                ``config.default_move_time``. **If > 0 it OVERRIDES ``speed`` and
                ``accel``** (URScript ``movel`` behavior). ``0.0`` lets
                ``speed``/``accel`` govern. Must be >= 0.

        Raises:
            SpeedViolation: If ``speed`` is not strictly positive (nothing is
                sent).
            ValueError: If ``move_time`` is negative.
        """
        speed = self._config.default_speed if speed is None else speed
        accel = self._config.default_accel if accel is None else accel
        t = self._resolve_move_time(move_time)
        vel = clamp_speed(speed, self._config)

        payload = [0.0] * _PAYLOAD_LEN
        msg = self.encode_command(_CMD_HOME, payload, accel, vel, t)
        self._conn.send(msg)

        if blocking:
            # The daemon homes to config.home_pose (already UR base frame).
            self._await_convergence(list(self._config.home_pose))

    # ------------------------------------------------------------------
    # Velocity / force primitives (thin encode+send; the daemon does the work)
    # ------------------------------------------------------------------
    def speed_l(
        self,
        velocity: Sequence[float],
        accel: Optional[float] = None,
        watchdog_t: float = _DEFAULT_SPEEDL_WATCHDOG_S,
    ) -> None:
        """Command a Cartesian TCP velocity (cmd=5, ``speedl``); non-blocking.

        The robot accelerates to ``velocity`` and holds it until a new command
        arrives or ``watchdog_t`` elapses (then it stops on its own — the
        hardware dead-man). The caller is responsible for monitoring and sending
        :meth:`stop` (this is how :meth:`ForceController.guarded_move` works).

        Args:
            velocity: TCP velocity ``[vx, vy, vz, wx, wy, wz]`` (m/s, rad/s),
                UR base frame.
            accel: Acceleration of the speed ramp (m/s^2). ``None`` uses
                ``config.default_accel``.
            watchdog_t: Seconds the robot keeps the velocity without a new command
                before auto-stopping.
        """
        velocity = list(velocity)
        if len(velocity) != _PAYLOAD_LEN:
            raise ValueError(f"velocity must have {_PAYLOAD_LEN} values, got {len(velocity)}")
        accel = self._config.default_accel if accel is None else accel
        self._conn.send(self.encode_command(_CMD_SPEEDL, velocity, accel, 0.0, watchdog_t))

    def force_push(
        self,
        direction_unit: Sequence[float],
        target_n: float,
        speed_limit: float,
        max_travel: float,
        accel: Optional[float] = None,
        frame_flag: float = 0.0,
    ) -> None:
        """Maintain a constant contact force (cmd=4 persistent force_mode); non-blocking.

        Sends the force-mode command; the controller then regulates the wrench at
        500 Hz until :meth:`end_force` (or any other command) arrives. **Gated on
        the controller by FORCE_MODE_ENABLED — pending hardware validation.**

        The 6 payload slots are full, so the **frame selector rides in the unused
        ``vel`` field** of the tuple: ``frame_flag`` 0.0 = base frame, 1.0 = tool
        frame. The daemon uses it to pick ``force_mode``'s task frame, so
        ``direction_unit`` is interpreted in that frame.

        Args:
            direction_unit: Push direction unit 3-vector ``[dx, dy, dz]`` in the
                frame selected by ``frame_flag`` (base or tool).
            target_n: Target contact force (N) along the direction.
            speed_limit: Compliant-axis speed cap (m/s).
            max_travel: Compliant-axis travel cap (m).
            accel: Ramp accel (m/s^2); ``None`` uses ``config.default_accel``.
            frame_flag: 0.0 = base frame (default), 1.0 = tool frame. Carried in
                the ``vel`` field (unused for force_mode commands).
        """
        d = list(direction_unit)
        payload = [d[0], d[1], d[2], float(target_n), float(speed_limit), float(max_travel)]
        accel = self._config.default_accel if accel is None else accel
        self._conn.send(
            self.encode_command(
                _CMD_FORCE, payload, accel, float(frame_flag), self._config.default_move_time
            )
        )

    def impedance_hold(
        self,
        compliant_axes: Sequence[float],
        stiffness: float,
        speed_limit: float,
        max_deviation: float,
        accel: Optional[float] = None,
        frame_flag: float = 0.0,
    ) -> None:
        """Hold a compliant spring about the current pose (cmd=7); non-blocking.

        The controller holds the entry pose as equilibrium and yields to external
        force with finite stiffness until :meth:`end_force`. **Gated on the
        controller by FORCE_MODE_ENABLED — pending hardware validation.**

        Like :meth:`force_push`, the **frame selector rides in the unused ``vel``
        field**: ``frame_flag`` 0.0 = base, 1.0 = tool. It selects the frame whose
        axes ``compliant_axes`` refer to (base axes, or the tool axes frozen at the
        entry pose).

        Args:
            compliant_axes: 3 flags ``[cx, cy, cz]`` (1 = compliant, 0 = stiff),
                referring to the frame chosen by ``frame_flag``.
            stiffness: Spring stiffness K (N/m).
            speed_limit: Compliant-axis speed cap (m/s).
            max_deviation: Max deviation from equilibrium (m).
            accel: Ramp accel (m/s^2); ``None`` uses ``config.default_accel``.
            frame_flag: 0.0 = base frame (default), 1.0 = tool frame (entry pose).
                Carried in the ``vel`` field (unused for force_mode commands).
        """
        c = list(compliant_axes)
        payload = [c[0], c[1], c[2], float(stiffness), float(speed_limit), float(max_deviation)]
        accel = self._config.default_accel if accel is None else accel
        self._conn.send(
            self.encode_command(
                _CMD_IMPEDANCE, payload, accel, float(frame_flag), self._config.default_move_time
            )
        )

    def impedance_move(
        self,
        target: Sequence[float],
        stiffness: float,
        speed_limit: float,
        max_deviation: float,
        accel: Optional[float] = None,
    ) -> None:
        """Impedance spring toward a target position (cmd=8); non-blocking.

        Like :meth:`impedance_hold`, but the spring's equilibrium is the **target**
        rather than the entry pose: the controller pulls the TCP toward ``target``
        with force ``K * (target - x)`` (clamped per axis to ``K * max_deviation``)
        while still yielding to external force. All three translation axes comply;
        orientation is held at entry. Runs until :meth:`end_force`. **Gated on the
        controller by FORCE_MODE_ENABLED — pending hardware validation.**

        Args:
            target: Equilibrium position ``[x, y, z]`` (m, UR base frame).
            stiffness: Spring stiffness K (N/m).
            speed_limit: Compliant-axis speed cap (m/s).
            max_deviation: Per-axis error clamp (m); the spring force saturates at
                ``K * max_deviation`` so a far target cannot yank the arm.
            accel: Ramp accel (m/s^2); ``None`` uses ``config.default_accel``.

        Raises:
            ValueError: If ``target`` is not a 3-vector.
        """
        t = list(target)
        if len(t) != _VEC3_LEN:
            raise ValueError(f"target must have {_VEC3_LEN} values [x,y,z], got {len(t)}")
        payload = [t[0], t[1], t[2], float(stiffness), float(speed_limit), float(max_deviation)]
        accel = self._config.default_accel if accel is None else accel
        self._conn.send(
            self.encode_command(
                _CMD_IMPEDANCE_MOVE, payload, accel, 0.0, self._config.default_move_time
            )
        )

    def end_force(self) -> None:
        """Exit any active force/impedance mode and hold the pose (cmd=6)."""
        self._conn.send(self.encode_command(_CMD_END_FORCE, [0.0] * _PAYLOAD_LEN, 0.0, 0.0, 0.0))

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _resolve_move_time(self, move_time: Optional[float]) -> float:
        """Resolve the URScript ``t`` for a move (default-or-explicit, validated).

        Returns ``config.default_move_time`` when ``move_time`` is ``None``,
        otherwise the explicit value. A non-zero result OVERRIDES speed/accel on
        the controller (URScript ``movel``/``movej`` semantics), so callers pass
        ``0.0`` to keep speed authoritative or a positive duration to pin the move
        length.

        Args:
            move_time: Requested duration in seconds, or ``None`` for the config
                default.

        Returns:
            The duration in seconds to encode as ``t``.

        Raises:
            ValueError: If ``move_time`` is negative.
        """
        t = self._config.default_move_time if move_time is None else float(move_time)
        if t < 0.0:
            raise ValueError(f"move_time must be >= 0 s, got {t}")
        return t

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
        seconds and returns once **both**:

        * the position error ``max(abs(dx, dy, dz))`` is below
          :attr:`RobotConfig.convergence_tol` (meters), and
        * the **orientation angle** between the actual and target rotation vectors
          is below ``convergence_tol`` (radians).

        Orientation is compared by ANGLE (via :func:`_orientation_angle`), not by
        rotation-vector components: near 180 degrees the controller may report the
        equivalent "wrapped" rotation vector (axis-angle double cover), e.g. a
        target ``[..,-3.14, 0.5]`` reported as ``[.., 3.06, -0.49]`` — the same
        pose, but a component-wise compare would never converge and the move would
        falsely time out.

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
                    pos_err = max(
                        abs(state.tcp_pose[i] - ur_target[i]) for i in range(3)
                    )
                    ang_err = _orientation_angle(state.tcp_pose[3:6], ur_target[3:6])
                    if pos_err < tol and ang_err < tol:
                        return
            time.sleep(_POLL_INTERVAL_S)
        raise TimeoutError(
            f"move did not converge within {_MAX_POLL_ITERS} polls "
            f"(tol={tol} m/rad)"
        )

    def _await_joint_convergence(self, target_joints: Sequence[float]) -> None:
        """Block until the joints are within ``convergence_tol`` of ``target_joints``.

        The state stream carries the actual joint angles, and a joint move's
        target IS known, so (just like :meth:`_await_convergence` for Cartesian
        moves) we wait until every joint reaches its commanded value. This is
        robust at the *start* of a move: an earlier "wait until the pose stops
        changing" heuristic could return immediately because the arm had barely
        begun accelerating, letting the next command pre-empt the unfinished
        joint move.

        Args:
            target_joints: Commanded joint angles ``[j0..j5]`` in radians.

        Raises:
            TimeoutError: If the joints do not converge within ``_MAX_POLL_ITERS``
                poll iterations.
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
                        abs(state.joints[i] - target_joints[i])
                        for i in range(_PAYLOAD_LEN)
                    )
                    if delta < tol:
                        return
            time.sleep(_POLL_INTERVAL_S)
        raise TimeoutError(
            f"joint move did not converge within {_MAX_POLL_ITERS} polls "
            f"(tol={tol} rad)"
        )
