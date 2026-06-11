"""Force-guided approach control for the UR5e (move-until-force, then hold).

.. warning::

   **Pending hardware validation / tuning.** This controller is a *pure control
   law* exercised today only with a mock motion controller and a scripted
   :class:`~ur5e_control.force.sensor.MockForceSensor`. No part of it has been
   run against a physically mounted Robotiq FT 300. The step size, the
   force-magnitude threshold behaviour near contact, the ``stiffness`` value,
   and the ``max_travel`` safety budget **must be re-tuned on the real robot**
   before any contact task is trusted. Until then treat all numeric defaults as
   placeholders.

:class:`ForceController` implements ``approach_until_force``: drive the TCP in
small relative linear steps along a commanded direction while watching the
force/torque sensor, and the moment the measured force **projected onto that
direction** reaches the target, stop approaching and transition to a held
URScript *force-mode* command (``cmd=4``). The hold tuple is encoded through the
motion controller's :meth:`~ur5e_control.motion.MotionController.encode_command`
and carries the unit direction, target force, stiffness, and max-travel exactly
as the PC -> daemon protocol specifies.

Units & frames (matching the rest of the library):

* The approach ``direction`` is a 3-vector in the **UR base frame** (the same
  frame the sensor wrench and TCP pose live in); it is normalised internally so
  its magnitude is irrelevant.
* ``target_n`` is a force in **newtons (N)**; ``max_travel`` and the internal
  step size are in **meters (m)**; ``stiffness`` is the force-mode compliance
  parameter handed straight to the daemon.
* Sensor wrenches are ``[fx, fy, fz, tx, ty, tz]`` (N, Nm), UR base frame; only
  the linear force triplet ``[fx, fy, fz]`` is used here.

Force-mode command (PC -> daemon), ``cmd=4`` force::

    "(4, dx, dy, dz, target_n, stiffness, max_travel, accel, vel, time)"

where ``(dx, dy, dz)`` is the *unit* approach direction, ``target_n`` the target
force (N), ``stiffness`` the compliance, and ``max_travel`` the travel ceiling
(m). This matches ``encode_command(4, [dx, dy, dz, target_n, stiffness,
max_travel], accel, vel, time)``.
"""

from __future__ import annotations

import math
import time
from typing import Sequence

from ..config import RobotConfig
from ..force.sensor import ForceSensor

__all__ = ["ForceController"]

# Force-mode opcode in the PC -> daemon tuple (see module docstring / protocol).
_CMD_FORCE = 4

# Length of a Cartesian direction vector / force triplet.
_VEC3_LEN = 3

# Length of the encode_command payload (a0..a5).
_PAYLOAD_LEN = 6

# Default per-iteration approach step (meters). PLACEHOLDER — pending on-robot
# tuning. Small so the move-until-force law does not overshoot the contact step.
_DEFAULT_APPROACH_STEP_M = 0.005

# Hard ceiling on approach iterations, independent of ``max_travel``, so a sensor
# that never reaches the target cannot wedge the caller forever.
_MAX_APPROACH_ITERS = 100000


class ForceController:
    """Move-until-force-then-hold controller built on a motion controller + sensor.

    The controller is deliberately transport-agnostic and fully mockable: it
    depends only on a ``motion`` object exposing
    :meth:`~ur5e_control.motion.MotionController.move_l`,
    :meth:`~ur5e_control.motion.MotionController.encode_command`, and a way to
    push a raw command string (a public ``send_command`` if present, otherwise
    the underlying connection's ``send``), and on a
    :class:`~ur5e_control.force.sensor.ForceSensor` exposing ``read()``.

    Args:
        motion: Motion controller providing ``move_l(pose, ..., relative=...)``,
            the static ``encode_command(cmd, payload, accel, vel, time)``, and a
            command-send path. Approach steps are issued as relative linear moves
            in the UR base frame (meters/radians).
        sensor: Force/torque sensor whose ``read()`` returns a 6-element wrench
            ``[fx, fy, fz, tx, ty, tz]`` (N, Nm), UR base frame.
        config: Robot configuration supplying motion defaults (speed/accel/time).
            Defaults to a fresh :class:`RobotConfig`.

    .. warning::
       Pending hardware validation / tuning — see the module docstring. The
       control law is verified only against mocks while no FT 300 is mounted.
    """

    def __init__(
        self,
        motion,
        sensor: ForceSensor,
        config: RobotConfig = RobotConfig(),
    ) -> None:
        self._motion = motion
        self._sensor = sensor
        self._config = config

    # ------------------------------------------------------------------
    # Public control law
    # ------------------------------------------------------------------
    def approach_until_force(
        self,
        direction: Sequence[float],
        target_n: float,
        stiffness: float,
        max_travel: float,
        step: float = _DEFAULT_APPROACH_STEP_M,
    ) -> None:
        """Approach along ``direction`` until contact, then hold with force mode.

        Drives the TCP in small relative linear steps along the (normalised)
        ``direction`` while polling ``sensor.read()`` on every iteration. The
        decision variable is the measured force **projected onto the unit
        direction** (``f . d_hat``, in newtons) — perpendicular forces do not
        trigger contact. The instant that projection reaches ``target_n``, the
        approach stops **immediately** (no further step is issued, so the contact
        step is never overshot) and a held force-mode command (``cmd=4``) is sent.

        The held command encodes, via the motion controller's
        ``encode_command``, ``a0..a2 = unit direction``, ``a3 = target_n``,
        ``a4 = stiffness``, ``a5 = max_travel`` (see the module docstring for the
        exact tuple), with the configured default accel/vel/move-time.

        Args:
            direction: Approach direction as a 3-vector ``[dx, dy, dz]`` in the
                UR base frame. Magnitude is irrelevant (it is normalised); only
                the orientation matters.
            target_n: Target contact force in **newtons** measured along
                ``direction``. Must be strictly positive.
            stiffness: Force-mode compliance/stiffness parameter passed straight
                through to the daemon's force mode.
            max_travel: Maximum distance to travel along ``direction`` while
                searching for contact, in **meters**. Must be strictly positive;
                if contact is not reached within this budget the approach aborts
                (no force-mode command is sent).
            step: Per-iteration approach increment in meters (PLACEHOLDER default,
                pending on-robot tuning). Must be strictly positive.

        Raises:
            ValueError: If ``direction`` is not a non-zero 3-vector, or if
                ``target_n``, ``max_travel``, or ``step`` is not strictly
                positive.
            RuntimeError: If ``max_travel`` is consumed (or the iteration ceiling
                is hit) before the target force is reached. No force-mode hold is
                emitted in that case.
        """
        unit = self._unit_direction(direction)

        if target_n <= 0.0:
            raise ValueError(f"target_n must be > 0 N, got {target_n}")
        if max_travel <= 0.0:
            raise ValueError(f"max_travel must be > 0 m, got {max_travel}")
        if step <= 0.0:
            raise ValueError(f"step must be > 0 m, got {step}")

        travelled = 0.0
        iters = 0
        while True:
            # Check contact BEFORE moving so we stop/hold exactly at the contact
            # step and never issue an approach step past it (no overshoot).
            wrench = self._sensor.read()
            force_along = self._project(wrench, unit)
            if force_along >= target_n:
                self._engage_force_mode(unit, target_n, stiffness, max_travel)
                return

            # Not in contact yet: refuse to step past the travel budget.
            if travelled + step > max_travel:
                raise RuntimeError(
                    f"approach exhausted max_travel ({max_travel} m) before "
                    f"reaching target force ({target_n} N); no contact detected"
                )

            iters += 1
            if iters > _MAX_APPROACH_ITERS:
                raise RuntimeError(
                    f"approach exceeded {_MAX_APPROACH_ITERS} iterations without "
                    f"reaching target force ({target_n} N)"
                )

            # One relative step along the unit direction (rotation untouched).
            delta_pose = [unit[0] * step, unit[1] * step, unit[2] * step, 0.0, 0.0, 0.0]
            self._motion.move_l(
                delta_pose,
                speed=self._config.default_speed,
                accel=self._config.default_accel,
                blocking=True,
                relative=True,
            )
            travelled += step

    # ------------------------------------------------------------------
    # Behavior 1 — GUARDED MOVE (velocity, PC-side): enabled
    # ------------------------------------------------------------------
    def guarded_move(
        self,
        direction: Sequence[float],
        speed: float,
        force_threshold_n: float,
        max_travel: float,
        accel=None,
        poll_interval: float = 0.02,
    ) -> list[float]:
        """Move along ``direction`` until contact, then STOP and hold the pose.

        PC-side velocity guarded move: command a single Cartesian velocity
        (``cmd=5`` ``speedl``), monitor the wrench, and the instant the force
        **projected onto the direction** reaches ``force_threshold_n``, send a
        stop (``cmd=2``) — the robot holds the contact pose stiffly. This is a
        threshold detector (not a force loop), so the PC poll rate is plenty.

        Safety: the velocity is sent with a ``speedl`` watchdog of
        ``max_travel/speed + margin`` seconds, so if this process dies the robot
        stops on its own. ``max_travel`` (as a constant-velocity time budget) is
        the PC-side cap; exceeding it stops and raises.

        Args:
            direction: Approach direction 3-vector ``[dx, dy, dz]`` (UR base
                frame); normalised internally.
            speed: Approach speed along the direction (m/s, > 0).
            force_threshold_n: Contact force (N) along the direction that triggers
                the stop (> 0).
            max_travel: Max distance to search for contact (m, > 0).
            accel: Speed-ramp acceleration (m/s^2); ``None`` uses the config
                default.
            poll_interval: Seconds between wrench reads.

        Returns:
            The contact wrench ``[fx, fy, fz, tx, ty, tz]`` at the moment of stop.

        Raises:
            ValueError: For a bad direction or non-positive speed/threshold/travel.
            RuntimeError: If ``max_travel`` is consumed before contact (the robot
                is stopped first).
        """
        unit = self._unit_direction(direction)
        if speed <= 0.0:
            raise ValueError(f"speed must be > 0 m/s, got {speed}")
        if force_threshold_n <= 0.0:
            raise ValueError(f"force_threshold_n must be > 0 N, got {force_threshold_n}")
        if max_travel <= 0.0:
            raise ValueError(f"max_travel must be > 0 m, got {max_travel}")

        velocity = [unit[0] * speed, unit[1] * speed, unit[2] * speed, 0.0, 0.0, 0.0]
        max_time = max_travel / speed
        self._motion.speed_l(velocity, accel=accel, watchdog_t=max_time + 0.5)

        elapsed = 0.0
        while elapsed <= max_time:
            wrench = self._sensor.read()
            # Use the |force| along the axis: a contact reaction opposes the motion
            # so the signed projection can be negative; magnitude is sign-convention
            # robust (works whether the wrench is env-on-robot or robot-on-env).
            if abs(self._project(wrench, unit)) >= force_threshold_n:
                self._motion.stop()
                return list(wrench)
            time.sleep(poll_interval)
            elapsed += poll_interval

        self._motion.stop()
        raise RuntimeError(
            f"guarded_move: no contact within max_travel ({max_travel} m) "
            f"at {force_threshold_n} N"
        )

    # ------------------------------------------------------------------
    # Behaviors 2 & 3 — force_mode on the controller: PENDING HARDWARE VALIDATION
    # (gated on the robot by FORCE_MODE_ENABLED; these just send the command)
    # ------------------------------------------------------------------
    def maintain_force(
        self,
        direction: Sequence[float],
        target_n: float,
        speed_limit: float = 0.05,
        max_travel: float = 0.05,
        accel=None,
    ) -> None:
        """Regulate a constant contact force ``target_n`` N along ``direction``.

        Hands off to the controller's native ``force_mode`` (``cmd=4``), which
        regulates the wrench at 500 Hz and follows the surface until
        :meth:`end_force`. Non-blocking. **PENDING HARDWARE VALIDATION** — gated
        on the robot by ``FORCE_MODE_ENABLED`` (a no-op until you enable it).

        Args:
            direction: Push direction 3-vector (UR base frame); normalised.
            target_n: Target contact force (N, > 0).
            speed_limit: Compliant-axis speed cap (m/s).
            max_travel: Compliant-axis travel cap (m).
            accel: Ramp accel (m/s^2); ``None`` uses the config default.
        """
        unit = self._unit_direction(direction)
        if target_n <= 0.0:
            raise ValueError(f"target_n must be > 0 N, got {target_n}")
        self._motion.force_push(unit, target_n, speed_limit, max_travel, accel=accel)

    def hold_compliant(
        self,
        compliant_axes: Sequence[float] = (1, 1, 1),
        stiffness: float = 300.0,
        speed_limit: float = 0.05,
        max_deviation: float = 0.05,
        accel=None,
    ) -> None:
        """Hold a compliant spring about the current pose (impedance).

        Hands off to the controller's ``force_mode`` configured as a spring
        (``cmd=7``): the entry pose is the equilibrium, and the arm yields to
        external force with finite ``stiffness`` until :meth:`end_force`.
        Non-blocking. **PENDING HARDWARE VALIDATION** — gated on the robot by
        ``FORCE_MODE_ENABLED``.

        Args:
            compliant_axes: 3 flags ``[cx, cy, cz]`` (1 = compliant, 0 = stiff).
            stiffness: Spring stiffness K (N/m, > 0).
            speed_limit: Compliant-axis speed cap (m/s).
            max_deviation: Max deviation from equilibrium (m).
            accel: Ramp accel (m/s^2); ``None`` uses the config default.
        """
        axes = list(compliant_axes)
        if len(axes) != _VEC3_LEN:
            raise ValueError(f"compliant_axes must have {_VEC3_LEN} flags, got {len(axes)}")
        if stiffness <= 0.0:
            raise ValueError(f"stiffness must be > 0 N/m, got {stiffness}")
        self._motion.impedance_hold(axes, stiffness, speed_limit, max_deviation, accel=accel)

    def end_force(self) -> None:
        """Exit any active force/impedance mode and hold the pose (``cmd=6``)."""
        self._motion.end_force()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _unit_direction(direction: Sequence[float]) -> list[float]:
        """Normalise a 3-vector approach direction (UR base frame).

        Args:
            direction: ``[dx, dy, dz]`` in the UR base frame; any non-zero
                magnitude is accepted.

        Returns:
            A fresh unit-length ``[dx, dy, dz]`` list.

        Raises:
            ValueError: If ``direction`` is not exactly 3 elements or has zero
                magnitude (no defined direction).
        """
        vec = [float(v) for v in direction]
        if len(vec) != _VEC3_LEN:
            raise ValueError(
                f"direction must have {_VEC3_LEN} elements [dx,dy,dz], got {len(vec)}"
            )
        norm = math.sqrt(vec[0] ** 2 + vec[1] ** 2 + vec[2] ** 2)
        if norm == 0.0:
            raise ValueError("direction must be a non-zero vector")
        return [vec[0] / norm, vec[1] / norm, vec[2] / norm]

    @staticmethod
    def _project(wrench: Sequence[float], unit: Sequence[float]) -> float:
        """Project a wrench's force triplet onto a unit direction (newtons).

        Args:
            wrench: 6-element ``[fx, fy, fz, tx, ty, tz]`` (N, Nm), UR base frame;
                only ``[fx, fy, fz]`` is used.
            unit: Unit direction ``[dx, dy, dz]`` (UR base frame).

        Returns:
            The signed scalar force component along ``unit`` in newtons
            (``f . d_hat``). Positive means pushing back along the approach axis.
        """
        return wrench[0] * unit[0] + wrench[1] * unit[1] + wrench[2] * unit[2]

    def _engage_force_mode(
        self,
        unit: Sequence[float],
        target_n: float,
        stiffness: float,
        max_travel: float,
    ) -> None:
        """Encode and send the held force-mode command (``cmd=4``).

        Builds the protocol tuple ``(4, dx, dy, dz, target_n, stiffness,
        max_travel, accel, vel, time)`` via the motion controller's
        ``encode_command`` and pushes it through the motion controller's command
        path (a public ``send_command`` if present, else the underlying
        connection's ``send``).

        Args:
            unit: Unit approach direction ``[dx, dy, dz]`` (UR base frame).
            target_n: Target hold force in newtons.
            stiffness: Force-mode compliance parameter.
            max_travel: Max travel ceiling in meters.
        """
        payload = [
            unit[0],
            unit[1],
            unit[2],
            float(target_n),
            float(stiffness),
            float(max_travel),
        ]
        msg = self._motion.encode_command(
            _CMD_FORCE,
            payload,
            self._config.default_accel,
            self._config.default_speed,
            self._config.default_move_time,
        )
        self._send(msg)

    def _send(self, msg: str) -> None:
        """Push a raw command string through the motion controller's transport.

        Prefers a public ``send_command`` on the motion controller (used by the
        mock in tests); falls back to the underlying connection's ``send`` so the
        real :class:`~ur5e_control.motion.MotionController` works unchanged.

        Args:
            msg: The encoded command tuple string to transmit verbatim.

        Raises:
            AttributeError: If the motion controller exposes neither a
                ``send_command`` method nor a connection with ``send``.
        """
        send_command = getattr(self._motion, "send_command", None)
        if callable(send_command):
            send_command(msg)
            return
        conn = getattr(self._motion, "_conn", None)
        if conn is not None and hasattr(conn, "send"):
            conn.send(msg)
            return
        raise AttributeError(
            "motion controller exposes no send path (need 'send_command' or "
            "a connection with 'send')"
        )
