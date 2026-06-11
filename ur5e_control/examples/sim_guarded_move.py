"""Demo: guarded_move in the MuJoCo UR5e sim (no real robot).

Drives the official menagerie UR5e down onto a table until the wrist force
reaches a threshold, then stops and holds — the same library code that would run
on hardware, against physics.

    python -m ur5e_control.examples.sim_guarded_move           # headless
    python -m ur5e_control.examples.sim_guarded_move --view    # open the viewer

Needs MuJoCo: ``uv pip install mujoco`` (or ``pip install ur5e-control[sim]``).
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

_ROOT = str(Path(__file__).resolve().parents[2])
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from ur5e_control import RobotConfig, UR5eRobot
from ur5e_control.sim import SimConnection


def main(view: bool = False) -> None:
    sim = SimConnection(realtime=True, viewer=view)
    robot = UR5eRobot(RobotConfig(), connection=sim)  # inject the sim transport
    with robot:
        if not robot.wait_until_connected(timeout=5.0):
            print("sim did not start streaming")
            return
        print("home joints:", [round(v, 3) for v in robot.get_state().joints])
        print("moving down (-Z, base) at 3 cm/s until |F| >= 20 N ...")
        contact = robot.force.guarded_move(
            direction=[0, 0, -1], speed=0.03, force_threshold_n=20.0, max_travel=0.15
        )
        print("contact wrench (base):", [round(v, 2) for v in contact])
        print("held pose  (base):", [round(v, 3) for v in robot.get_state().tcp_pose])
        if view:
            print("viewer open — Ctrl-C to quit.")
            try:
                while True:
                    time.sleep(1.0)
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main(view="--view" in sys.argv[1:])
