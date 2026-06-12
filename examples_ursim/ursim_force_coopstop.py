"""Validate the daemon's COOPERATIVE force-mode stop on URSim.

The dangerous bug a hard ``kill`` would cause: killing the control thread while
it is inside ``force_mode`` leaves the arm compliant (``end_force_mode`` never
runs). The daemon avoids this in ``client()``: when a new command arrives while a
force mode is running (``active_force != 0``), it sets ``force_stop``, waits for
the force loop to exit (which runs ``end_force_mode``), and only then dispatches
the new command.

URSim runs the REAL controller software, so ``force_mode`` / ``end_force_mode``
execute for real (URSim just has no contact/FT, so the force isn't felt). That is
enough to validate the *control flow*: enter ``maintain_force`` (cmd 4), then send
a ``moveJ`` and confirm the arm actually reaches the new target. If the move
completes, the force mode must have exited cooperatively first — a deadlock or a
stuck-compliant arm would never converge.

This uploads the daemon with ``force_mode_enabled=True`` (so cmd 4 is armed).

Prerequisites (same as ursim_validate.py):
  1. Docker installed; URSim e-series running with --network host.
  2. In PolyScope (http://127.0.0.1:6080/vnc.html): initialise the robot
     (power on + release brakes) and enable Remote Control.

Run:  python examples_ursim/ursim_force_coopstop.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

_ROOT = str(Path(__file__).resolve().parents[1])
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from ur5e_control import RobotConfig, UR5eRobot

# Same URSim networking as ursim_validate.py (callback on 40002 to dodge URSim's
# own 30002), but with force_mode ARMED so cmd 4 actually runs.
URSIM_FORCE_CONFIG = RobotConfig(
    controller_ip="127.0.0.1",
    script_port=30001,
    pc_host="127.0.0.1",
    state_port=40002,
    workspace_limits={"x": (-1.5, 1.5), "y": (-1.5, 1.5), "z": (-1.5, 1.5)},
    force_mode_enabled=True,
)

_SAFE_START = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
_MOVE_TARGET = [0.3, -1.2, 1.2, -1.57, -1.57, 0.0]   # clearly different from start
_CONVERGE_TOL = 0.05      # rad — "did the moveJ actually arrive?"
_CONVERGE_TIMEOUT_S = 15.0


def _await_joints(robot: UR5eRobot, target, timeout: float) -> bool:
    """Poll until every joint is within tolerance of target, or timeout."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            joints = robot.get_state().joints
            if max(abs(joints[i] - target[i]) for i in range(6)) < _CONVERGE_TOL:
                return True
        except ValueError:
            pass
        time.sleep(0.1)
    return False


def main() -> None:
    print("== URSim cooperative force-mode stop validation ==")
    robot = UR5eRobot(URSIM_FORCE_CONFIG)
    with robot:
        print("1) uploaded daemon (force_mode_enabled=True); waiting for callback ...")
        if not robot.wait_until_connected(timeout=15.0):
            print("   [FAIL] daemon did not connect back. Check Remote Control / "
                  "robot initialised / --network host / port 40002 free.")
            return
        print("   [OK] daemon connected.\n")

        print("2) moveJ to a safe start configuration ...")
        robot.move_j(_SAFE_START, speed=0.4)
        print("   joints:", [round(v, 3) for v in robot.get_state().joints], "\n")

        print("3) enter maintain_force (cmd 4) — arm becomes compliant on Z ...")
        robot.force.maintain_force(
            direction=[0.0, 0.0, -1.0], target_n=5.0,
            speed_limit=0.05, max_travel=0.05,
        )
        time.sleep(1.5)  # let force_mode actually run (active_force = 4)
        print("   force_mode running.\n")

        print("4) send moveJ WHILE force_mode runs — this must cooperatively stop "
              "force_mode (end_force_mode) before moving ...")
        robot.move_j(_MOVE_TARGET, speed=0.4, blocking=False)
        arrived = _await_joints(robot, _MOVE_TARGET, _CONVERGE_TIMEOUT_S)
        if arrived:
            print("   [OK] arm reached the new target -> force_mode exited "
                  "cooperatively and the move ran.\n")
        else:
            print("   [FAIL] arm did not reach the target in time -> cooperative "
                  "stop may be deadlocked or the arm stayed compliant.\n")

        print("5) end_force (belt-and-suspenders) + home ...")
        robot.force.end_force()
        robot.home(speed=0.4)
        time.sleep(0.3)
        print("   joints:", [round(v, 3) for v in robot.get_state().joints], "\n")

    verdict = "PASS" if arrived else "FAIL"
    print(f"== Cooperative force-mode stop on URSim: {verdict} ==")


if __name__ == "__main__":
    main()
