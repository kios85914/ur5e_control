"""High-level UR5e robot facade for the control library.

:class:`UR5eRobot` is the public, user-facing entry point. It composes the three
lower layers — the URScript daemon uploader (:mod:`ur5e_control.script_sender`),
the socket transport (:class:`~ur5e_control.connection.RobotConnection`), and the
motion encoder/executor (:class:`~ur5e_control.motion.MotionController`) — into a
single object with a small, stable API. It owns no geometry or protocol logic of
its own: every motion request is delegated to the :class:`MotionController`, and
state is parsed from the raw frame the connection last received.

Units and frames (matching the rest of the library):

* Positions are in **meters**, rotations/joints in **radians**.
* A *pose* is ``[x, y, z, rx, ry, rz]`` (axis-angle orientation); *joints* are
  ``[j0, j1, j2, j3, j4, j5]``.
* Cartesian inputs to :meth:`move_l` are in the **world frame**; the underlying
  :class:`MotionController` converts them to the **UR base frame** via
  :meth:`RobotConfig.world_to_ur`. Joint inputs to :meth:`move_j` are not
  frame-dependent. :class:`RobotState` returned by :meth:`get_state` is in the
  **UR base frame**.

Lifecycle:

* :meth:`connect` uploads the motion daemon to the controller (via
  :func:`~ur5e_control.script_sender.send_script`) and then starts the connection
  so the daemon's state stream begins flowing back. :meth:`disconnect` closes the
  connection.
* The object is also a context manager: ``with UR5eRobot(cfg) as robot:`` connects
  on entry and disconnects on exit (even if the body raises).

Typical use::

    from ur5e_control.config import RobotConfig
    from ur5e_control.robot import UR5eRobot

    with UR5eRobot(RobotConfig()) as robot:
        robot.move_l([0.1, 0.3, 0.2, 0.0, -3.14, 0.0])  # world frame, m/rad
        state = robot.get_state()                        # UR base frame
        robot.home()
"""

from __future__ import annotations

import time
from typing import Optional, Sequence

from .config import RobotConfig
from .connection import RobotConnection
from .motion import MotionController
from .script_sender import render_daemon, send_script
from .state import RobotState, parse_state

__all__ = ["UR5eRobot"]


class UR5eRobot:
    """Facade composing daemon upload, socket transport, and motion control.

    Construction wires up a :class:`~ur5e_control.connection.RobotConnection`
    (from the config) and a :class:`~ur5e_control.motion.MotionController` (from
    the connection and config) but performs no I/O. Networking begins only at
    :meth:`connect` (or context-manager entry).

    Args:
        config: Robot configuration supplying network endpoints, motion defaults,
            safety limits, and the world<->UR frame transform. Defaults to a fresh
            :class:`RobotConfig`.
        dry_run: When ``True``, no real sockets are opened and the daemon is not
            uploaded — :meth:`connect`/:meth:`disconnect` and every move just log.
            Useful for previewing commands or driving the GUI with no robot
            present. Defaults to ``False`` (talk to the real controller).
    """

    def __init__(
        self,
        config: RobotConfig = RobotConfig(),
        dry_run: bool = False,
        connection=None,
    ) -> None:
        self.config = config
        self.dry_run = dry_run
        # An injected transport (e.g. ur5e_control.sim.SimConnection) replaces the
        # real socket connection AND suppresses the URScript daemon upload, so the
        # whole library can drive a simulator. When None, talk to the real robot.
        self._injected = connection is not None
        self._connection = connection if self._injected else RobotConnection(config, dry_run=dry_run)
        self._motion = MotionController(self._connection, config)
        self._force = None  # lazily built ForceController (see the `force` property)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def connect(self) -> None:
        """Upload the motion daemon, then start the state connection.

        Renders the packaged URScript daemon with
        :func:`~ur5e_control.script_sender.render_daemon` (injecting
        ``pc_host``/``state_port``/``home_pose`` from the config), uploads it to
        the controller with :func:`~ur5e_control.script_sender.send_script` (which
        sends it to ``config.controller_ip:config.script_port``), and then starts
        the :class:`~ur5e_control.connection.RobotConnection` so the daemon's
        state stream is received. The upload must precede the connection start so
        the daemon is running before it tries to connect back to the PC.
        """
        # An injected transport (sim) runs no URScript -- just start it.
        if not self._injected:
            send_script(render_daemon(self.config), self.config, dry_run=self.dry_run)
        self._connection.start()

    def disconnect(self) -> None:
        """Close the state connection and release its sockets.

        Idempotent: safe to call more than once or before :meth:`connect`
        (delegates to :meth:`RobotConnection.close`, which is itself idempotent).
        """
        self._connection.close()

    def serve_gui(self, host: str = "127.0.0.1", port: int = 8080):
        """Open the browser monitor/control panel attached to this robot.

        One call, no extra wiring — keep your control code clean::

            with UR5eRobot(RobotConfig()) as robot:
                robot.serve_gui()                 # http://127.0.0.1:8080
                robot.move_l([0.1, 0.3, 0.2, 0, -3.14, 0])   # the GUI shows it live

        The GUI runs on a background thread (this returns immediately) and shares
        *this* robot, so your Python code keeps driving while the browser shows
        live state. It opens in **Python control mode**: the browser's
        move/jog/home are locked to a live monitor (so a human can't fight your
        script) while **STOP always works**; flip to "GUI control" in the page to
        jog manually.

        Args:
            host: Bind host (default localhost).
            port: Bind port (default 8080).

        Returns:
            The running ``ThreadingHTTPServer`` (call ``.shutdown()`` to stop).
        """
        # Lazy import: ur5e_control.gui.server imports UR5eRobot, so importing it
        # at module load time would create a cycle.
        from .gui.server import serve_in_background

        return serve_in_background(self, host=host, port=port)

    # ------------------------------------------------------------------
    # Motion (delegated to MotionController)
    # ------------------------------------------------------------------
    def move_l(
        self,
        pose: Sequence[float],
        speed: Optional[float] = None,
        accel: Optional[float] = None,
        blocking: bool = True,
        relative: bool = False,
    ) -> None:
        """Move the TCP linearly to a Cartesian pose (delegates to MotionController).

        Args:
            pose: World-frame target (or delta if ``relative``)
                ``[x, y, z, rx, ry, rz]`` in meters/radians.
            speed: Cartesian speed in m/s. ``None`` uses ``config.default_speed``.
            accel: Cartesian acceleration in m/s^2. ``None`` uses
                ``config.default_accel``.
            blocking: If ``True``, block until the move converges on the target.
            relative: If ``True``, ``pose`` is a delta on the current world pose.
        """
        self._motion.move_l(
            pose, speed=speed, accel=accel, blocking=blocking, relative=relative
        )

    def move_j(
        self,
        joints: Sequence[float],
        speed: Optional[float] = None,
        accel: Optional[float] = None,
        blocking: bool = True,
    ) -> None:
        """Move to a joint configuration (delegates to MotionController).

        Args:
            joints: Target joint angles ``[j0..j5]`` in radians (frame-independent).
            speed: Joint speed in rad/s. ``None`` uses ``config.default_speed``.
            accel: Joint acceleration in rad/s^2. ``None`` uses
                ``config.default_accel``.
            blocking: If ``True``, block until the joint move settles.
        """
        self._motion.move_j(joints, speed=speed, accel=accel, blocking=blocking)

    def stop(self) -> None:
        """Command an immediate controlled stop (cmd=2, via MotionController)."""
        self._motion.stop()

    def home(
        self,
        speed: Optional[float] = None,
        accel: Optional[float] = None,
        blocking: bool = True,
    ) -> None:
        """Move to the configured home pose (cmd=3, via MotionController).

        The daemon homes to ``config.home_pose`` (UR base frame, meters/radians).

        Args:
            speed: Speed in m/s. ``None`` uses ``config.default_speed``.
            accel: Acceleration in m/s^2. ``None`` uses ``config.default_accel``.
            blocking: If ``True``, block until converged on the home pose.
        """
        self._motion.home(speed=speed, accel=accel, blocking=blocking)

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------
    @property
    def force(self) -> "ForceController":
        """Force-aware control bound to this robot (lazy, cached).

        Reads the live wrench from this robot's state stream (the Robotiq FT 300
        value the daemon already streams) and drives motion through this robot::

            robot.force.guarded_move([0, 0, -1], speed=0.02,
                                     force_threshold_n=10.0, max_travel=0.1)

        ``guarded_move`` (move until force, then stop & hold) is ready to use.
        ``maintain_force`` / ``hold_compliant`` use the controller's force_mode and
        are **gated on the robot by FORCE_MODE_ENABLED — pending hardware
        validation**.
        """
        if self._force is None:
            from .force.controller import ForceController
            from .force.sensor import RobotiqFT300

            sensor = RobotiqFT300(state_provider=self.get_state)
            self._force = ForceController(self._motion, sensor, self.config)
        return self._force

    def is_daemon_connected(self) -> bool:
        """Return ``True`` if the URScript daemon has connected back to the PC.

        Distinct from having called :meth:`connect`: after ``connect()`` the PC is
        listening, but the daemon on the controller may not have dialed back yet.
        Always ``False`` in ``dry_run`` mode. Useful for a UI status indicator.
        """
        return self._connection.is_connected()

    def wait_until_connected(self, timeout: float = 5.0, poll: float = 0.1) -> bool:
        """Block until the daemon has connected back to the PC, or ``timeout``.

        :meth:`connect` returns as soon as the PC is *listening*; the daemon on
        the controller then dials back a moment later. Call this to wait for that
        callback so you can confirm the robot is really up before commanding it::

            robot.connect()
            if robot.wait_until_connected():
                print("robot connected")

        Returns ``True`` once :meth:`is_daemon_connected` is true. In ``dry_run``
        there is no real daemon, so this returns ``False`` immediately (check
        ``robot.dry_run`` to tell that apart from a real timeout). Returns
        ``False`` if the daemon has not connected within ``timeout`` seconds.

        Args:
            timeout: Max seconds to wait for the daemon callback.
            poll: Seconds between checks.

        Returns:
            ``True`` if the daemon connected within the timeout, else ``False``.
        """
        if self.dry_run:
            return False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._connection.is_connected():
                return True
            time.sleep(poll)
        return self._connection.is_connected()

    def get_state(self) -> RobotState:
        """Return the latest robot state as a parsed :class:`RobotState`.

        Reads the most recent raw state frame from the connection
        (:meth:`RobotConnection.latest_state`) and parses it with
        :func:`~ur5e_control.state.parse_state`. All quantities are in the **UR
        base frame**, meters/radians (velocities m/s, rad/s; forces N, Nm).

        Returns:
            The parsed :class:`RobotState` snapshot.

        Raises:
            ValueError: If no complete state frame is available yet or the latest
                frame is malformed (propagated from :func:`parse_state`).
        """
        return parse_state(self._connection.latest_state())

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------
    def __enter__(self) -> "UR5eRobot":
        """Connect on entry and return ``self``."""
        self.connect()
        return self

    def __exit__(self, *exc_info: object) -> None:
        """Disconnect on exit (runs even if the body raised)."""
        self.disconnect()
