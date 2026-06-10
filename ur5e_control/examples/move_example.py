"""Example: move the UR5e with :class:`~ur5e_control.robot.UR5eRobot`.

This script walks through the everyday Cartesian-motion API on the
:class:`~ur5e_control.robot.UR5eRobot` facade:

#. build a :class:`~ur5e_control.config.RobotConfig`,
#. enter the robot as a **context manager** (connect on entry, disconnect on
   exit, even if the body raises),
#. :meth:`~ur5e_control.robot.UR5eRobot.move_l` the TCP to a Cartesian pose,
#. :meth:`~ur5e_control.robot.UR5eRobot.get_state` to read the latest state, and
#. :meth:`~ur5e_control.robot.UR5eRobot.home` back to the configured home pose.

Units & frames (matching the library): poses are ``[x, y, z, rx, ry, rz]`` in
**meters / radians**. Cartesian inputs to ``move_l`` are in the **world frame**
and are converted to the UR base frame inside the library;
:class:`~ur5e_control.state.RobotState` is reported in the UR base frame.

Running it
----------
By default this runs in **dry-run** mode: no socket is opened, no script is
uploaded, and no real robot is commanded — every command is merely logged, so
you can preview exactly what would be sent. This makes the example safe to run on
a developer machine with no robot present::

    python -m ur5e_control.examples.move_example          # dry-run preview
    python ur5e_control/examples/move_example.py          # dry-run preview
    python -m ur5e_control.examples.move_example --live   # talk to a real robot

.. warning::
   Only pass ``--live`` when a UR5e is reachable at the configured
   ``controller_ip`` and the configured ``pc_host`` is this machine's address;
   ``--live`` will move real hardware.
"""

from __future__ import annotations

import logging
import sys
from pathlib import Path

# Allow running this file directly (``python ur5e_control/examples/move_example.py``):
# put the project root (three levels up: <root>/ur5e_control/examples/<file>) on
# sys.path so ``import ur5e_control`` resolves without an editable install.
_PROJECT_ROOT = str(Path(__file__).resolve().parents[2])
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from ur5e_control.config import RobotConfig
from ur5e_control.connection import RobotConnection
from ur5e_control.motion import MotionController
from ur5e_control.robot import UR5eRobot

# A demo target pose in the WORLD frame, [x, y, z, rx, ry, rz] (meters/radians).
# Chosen to lie inside the default workspace once converted to the UR base frame
# (world x/y are negated by RobotConfig.world_to_ur): UR pose ~ (0.06, -0.30, ...)
# is a safe, illustrative target rather than a tuned production waypoint.
_DEMO_POSE_WORLD = [-0.06, -0.30, 0.20, 0.0, -3.14, 0.0]


def _make_dry_run_robot(config: RobotConfig) -> UR5eRobot:
    """Build a :class:`UR5eRobot` wired for a no-socket dry-run preview.

    The robot is constructed normally, then its transport is swapped for a
    :class:`~ur5e_control.connection.RobotConnection` in ``dry_run`` mode (which
    logs instead of touching the network) and its
    :class:`~ur5e_control.motion.MotionController` is rebuilt on top of that
    dry-run transport. Its :meth:`~ur5e_control.robot.UR5eRobot.connect` /
    :meth:`~ur5e_control.robot.UR5eRobot.disconnect` are replaced with versions
    that only start/stop the dry-run transport (no URScript upload, no real
    socket), so the public context-manager / motion API can be exercised exactly
    as on real hardware while opening nothing.

    Args:
        config: Robot configuration (network endpoints, motion defaults, frame
            transform). Units are meters/radians; geometry is UR base frame.

    Returns:
        A :class:`UR5eRobot` whose every command is logged rather than sent.
    """
    robot = UR5eRobot(config)

    dry_conn = RobotConnection(config, dry_run=True)
    # Re-wire the facade's private collaborators onto the dry-run transport.
    motion = MotionController(dry_conn, config)
    robot._connection = dry_conn
    robot._motion = motion

    # The locked UR5eRobot.home() blocks by default, which would poll a state
    # stream that does not exist in dry-run (eventually timing out). Force the
    # home move non-blocking here so the preview just logs the home command.
    _blocking_home = motion.home

    def _home_nonblocking() -> None:
        """Send the home command without waiting on an (absent) state stream."""
        _blocking_home(blocking=False)

    motion.home = _home_nonblocking  # type: ignore[assignment]

    # Replace lifecycle so connect()/the context manager never upload a script
    # or open a real socket; they just bring the dry-run transport up/down.
    robot.connect = dry_conn.start  # type: ignore[assignment]
    robot.disconnect = dry_conn.close  # type: ignore[assignment]
    return robot


def main(dry_run: bool = True) -> None:
    """Run the move/get_state/home demo against a UR5e (or a dry-run preview).

    Args:
        dry_run: When ``True`` (the default), build a no-socket dry-run robot via
            :func:`_make_dry_run_robot` so the example previews the commands it
            would send without touching hardware. When ``False``, drive a real
            robot through :class:`UR5eRobot` over actual sockets.

    Notes:
        In dry-run mode the daemon state stream is absent, so this uses
        non-blocking moves (no convergence polling) and reports that no live
        state is available. With a real robot (``dry_run=False``) the moves block
        until convergence and ``get_state()`` returns the streamed state.
    """
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    config = RobotConfig()
    mode = "DRY-RUN (no sockets, nothing is moved)" if dry_run else "LIVE (real robot)"
    print(f"== UR5e move example — mode: {mode} ==")
    print(f"controller_ip={config.controller_ip}  pc_host={config.pc_host}")

    robot = _make_dry_run_robot(config) if dry_run else UR5eRobot(config)

    # In dry-run there is no state stream to converge against, so don't block on
    # convergence; on a real robot we want blocking moves.
    blocking = not dry_run

    with robot:  # connect() on entry, disconnect() on exit (even on error)
        print(f"\n1) move_l to world pose {_DEMO_POSE_WORLD} (m, rad)")
        robot.move_l(_DEMO_POSE_WORLD, blocking=blocking)

        print("\n2) get_state (UR base frame)")
        try:
            state = robot.get_state()
            print(f"   tcp_pose = {state.tcp_pose}")
            print(f"   joints   = {state.joints}")
            print(f"   wrench   = {state.wrench}")
        except ValueError as exc:
            # Expected in dry-run: no daemon, hence no state frame to parse.
            print(f"   (no live state available: {exc})")

        print(f"\n3) home to {config.home_pose} (UR base frame)")
        robot.home()

    print("\nDone. (Context manager disconnected the robot.)")


if __name__ == "__main__":
    # Default to the safe dry-run preview; opt into real hardware with --live.
    live = "--live" in sys.argv[1:]
    main(dry_run=not live)
