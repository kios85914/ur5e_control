"""Tutorial: control from Python while the browser GUI shows live state.

Your Python script owns the robot and drives it; the GUI attaches to the *same*
``UR5eRobot`` (same process, same socket — no port conflict) and acts as a live
monitor. The GUI opens in **Python control mode**: its move/jog/home buttons are
locked (so a human can't fight your script), but **STOP** always works and you
can flip to "GUI control" in the page to jog manually for debugging.

    python -m ur5e_control.examples.python_with_gui          # dry-run (no robot)
    python -m ur5e_control.examples.python_with_gui --live   # real robot

Then open http://127.0.0.1:8080 in a browser while this script runs.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

_ROOT = str(Path(__file__).resolve().parents[2])
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from ur5e_control import RobotConfig, UR5eRobot


def main(dry_run: bool = True, hold: bool = False) -> None:
    config = RobotConfig()
    blocking = not dry_run
    print(f"== Python control + live GUI — {'DRY-RUN' if dry_run else 'LIVE'} ==")

    robot = UR5eRobot(config, dry_run=dry_run)
    robot.connect()
    try:
        # One call attaches the monitor GUI to THIS robot (background thread).
        robot.serve_gui()  # http://127.0.0.1:8080  (Python control mode)
        if not dry_run:
            time.sleep(0.5)

        # ---- your control code; the GUI mirrors it live ----
        print("driving from Python (watch the browser)…")
        robot.move_l([-0.06, -0.30, 0.20, 0.0, -3.14, 0.0], speed=0.05, blocking=blocking)
        robot.move_l([0.0, 0.0, 0.05, 0, 0, 0], relative=True, speed=0.05, blocking=blocking)
        robot.home(blocking=blocking)

        if hold:
            print("done driving — GUI stays up; Ctrl-C to quit.")
            while True:
                time.sleep(1.0)
    finally:
        robot.disconnect()


if __name__ == "__main__":
    # Keep the process (and GUI) alive after the demo moves so you can browse.
    main(dry_run="--live" not in sys.argv[1:], hold=True)
