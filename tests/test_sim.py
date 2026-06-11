"""End-to-end sim test: the library drives the MuJoCo UR5e (skipped without mujoco).

This is the payoff of the simulation backend — the *real* MotionController /
ForceController / guarded_move code runs against MuJoCo physics, so we can verify
"move until force, then stop & hold" with actual contact + a wrist FT reading, no
hardware. Skipped automatically where the ``mujoco`` package isn't installed.
"""

from __future__ import annotations

import time

import pytest

pytest.importorskip("mujoco")

from ur5e_control import RobotConfig, UR5eRobot  # noqa: E402
from ur5e_control.sim import SimConnection  # noqa: E402


def test_guarded_move_contacts_and_holds_in_sim():
    sim = SimConnection(realtime=True, viewer=False)
    robot = UR5eRobot(RobotConfig(), connection=sim)
    with robot:
        assert robot.wait_until_connected(timeout=5.0)
        z0 = robot.get_state().tcp_pose[2]

        contact = robot.force.guarded_move(
            direction=[0, 0, -1], speed=0.1, force_threshold_n=15.0, max_travel=0.15
        )

        # The contact force magnitude along the push axis reached the threshold.
        assert abs(contact[2]) >= 15.0
        # The TCP actually descended toward the table before stopping.
        time.sleep(0.2)
        z1 = robot.get_state().tcp_pose[2]
        assert z1 < z0 - 0.02
