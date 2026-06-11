"""Tutorial: control the UR5e purely from Python (no GUI).

This is the "just Python" path — everything goes through the
:class:`~ur5e_control.robot.UR5eRobot` facade. It walks through the whole control
surface step by step so a student can copy/adapt each piece:

#. build a :class:`~ur5e_control.config.RobotConfig` (network + limits),
#. connect (uploads the daemon, starts the state stream),
#. read state (TCP pose / joints / wrench),
#. absolute Cartesian move (``move_l``),
#. relative Cartesian "jog" (``move_l(..., relative=True)``),
#. joint-space move (``move_j``),
#. go home, then disconnect.

Frames & units (the whole library): poses are ``[x, y, z, rx, ry, rz]`` in
**meters / radians**. ``move_l`` inputs are **world frame** (converted to the UR
base frame inside the library); ``get_state()`` reports the **raw UR base frame**.

Run it::

    python -m ur5e_control.examples.python_control          # dry-run (no robot)
    python -m ur5e_control.examples.python_control --live   # real robot

In dry-run nothing is sent and there is no state stream, so moves are issued
non-blocking and state reads simply report "no live state".
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

# Allow ``python ur5e_control/examples/python_control.py`` without an install.
_ROOT = str(Path(__file__).resolve().parents[2])
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from ur5e_control import RobotConfig, UR5eRobot


def show_state(robot: UR5eRobot) -> None:
    """Print the latest state, or note that none is available (dry-run)."""
    try:
        s = robot.get_state()
        print(f"   tcp_pose = {[round(v, 4) for v in s.tcp_pose]}")
        print(f"   joints   = {[round(v, 4) for v in s.joints]}")
        print(f"   wrench   = {[round(v, 2) for v in s.wrench]}")
    except ValueError as exc:
        print(f"   (no live state: {exc})")


def main(dry_run: bool = True) -> None:
    config = RobotConfig()
    blocking = not dry_run  # no state stream to converge against in dry-run
    mode = "DRY-RUN (nothing is moved)" if dry_run else "LIVE (real robot)"
    print(f"== Python control tutorial — {mode} ==")
    print(f"controller_ip={config.controller_ip}  pc_host={config.pc_host}\n")

    # connect() on enter, disconnect() on exit (even if the body raises).
    with UR5eRobot(config, dry_run=dry_run) as robot:
        if not dry_run:
            time.sleep(0.5)  # give the daemon a moment to start streaming state

        print("1) current state")
        show_state(robot)

        print("\n2) absolute move_l to a world pose")
        robot.move_l([-0.06, -0.30, 0.20, 0.0, -3.14, 0.0], speed=0.05, blocking=blocking)

        print("\n3) relative jog: +5 cm in Z")
        try:
            robot.move_l([0.0, 0.0, 0.05, 0, 0, 0], relative=True, speed=0.05, blocking=blocking)
        except ValueError as exc:
            # relative needs the current pose; in dry-run there is no state stream.
            print(f"   (skipped — relative needs a live current pose: {exc})")

        print("\n4) joint move (small wrist nudge from a safe pose)")
        robot.move_j([0.0, -1.57, 1.57, -1.57, -1.57, 0.0], speed=0.2, blocking=blocking)

        print("\n5) state again")
        show_state(robot)

        print("\n6) home")
        robot.home(blocking=blocking)

    print("\nDone. (Context manager disconnected the robot.)")


if __name__ == "__main__":
    main(dry_run="--live" not in sys.argv[1:])
