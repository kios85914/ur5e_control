"""Validate motion_daemon.script against URSim (the official UR offline simulator).

URSim runs the REAL UR controller software and executes URScript, so this checks
the things MuJoCo cannot: the daemon uploads + runs, connects back, the command
dispatch (moveL/moveJ/home) executes with real kinematics, and the state-frame
format matches the real controller. URSim has no contact/FT, so force behaviors
(guarded_move / force_mode) are NOT meaningfully testable here — use MuJoCo for
those.

Prerequisites (see the steps printed by the assistant):
  1. Docker installed; URSim e-series container running with --network host.
  2. In PolyScope (http://127.0.0.1:6080/vnc.html): initialise the robot
     (power on + release brakes) and enable Remote Control.

Run:  python examples_ursim/ursim_validate.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

_ROOT = str(Path(__file__).resolve().parents[1])
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from ur5e_control import RobotConfig, UR5eRobot

# URSim config:
#  - controller_ip/script_port -> upload the daemon to URSim's primary interface
#  - pc_host/state_port        -> where the daemon connects BACK (our listener).
#    state_port is 40002 (NOT 30002) so it doesn't collide with URSim's own
#    secondary interface on 30002 when the container uses --network host.
#  - wide workspace_limits      -> we're validating the daemon, not cell safety.
URSIM_CONFIG = RobotConfig(
    controller_ip="127.0.0.1",
    script_port=30001,
    pc_host="127.0.0.1",
    state_port=40002,
    workspace_limits={"x": (-1.5, 1.5), "y": (-1.5, 1.5), "z": (-1.5, 1.5)},
)


def main() -> None:
    print("== URSim daemon validation ==")
    robot = UR5eRobot(URSIM_CONFIG)
    with robot:
        print("1) uploaded daemon; waiting for it to connect back to the PC ...")
        if not robot.wait_until_connected(timeout=15.0):
            print(
                "   [FAIL] daemon did not connect back within 15 s.\n"
                "   Check: URSim Remote Control is ENABLED; the robot is initialised\n"
                "   (powered on + brakes released); the container uses --network host;\n"
                "   nothing else is bound to port 40002."
            )
            return
        print("   [OK] daemon is running on URSim and streaming state.\n")

        st = robot.get_state()
        print("2) state frame parsed OK")
        print("   joints:", [round(v, 3) for v in st.joints])
        print("   tcp_pose (base):", [round(v, 3) for v in st.tcp_pose])
        print("   wrench:", [round(v, 2) for v in st.wrench], "(URSim: no real contact/FT)\n")

        print("3) moveJ to a safe configuration ...")
        robot.move_j([0.0, -1.57, 1.57, -1.57, -1.57, 0.0], speed=0.4)
        print("   joints now:", [round(v, 3) for v in robot.get_state().joints], "\n")

        print("4) moveL relative (+2 cm in Z) ...")
        robot.move_l([0.0, 0.0, 0.02, 0.0, 0.0, 0.0], speed=0.05, relative=True)
        print("   tcp_pose now:", [round(v, 3) for v in robot.get_state().tcp_pose], "\n")

        print("5) home ...")
        robot.home(speed=0.4)
        time.sleep(0.3)
        print("   joints now:", [round(v, 3) for v in robot.get_state().joints], "\n")

    print("== Daemon validated on URSim: upload, callback, state format, "
          "moveJ/moveL/home all executed. ==")


if __name__ == "__main__":
    main()
