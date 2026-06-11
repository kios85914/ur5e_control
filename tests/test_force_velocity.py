"""Tests for the velocity / force-mode additions.

Behavior 1 (guarded_move) is a PC-side velocity loop and is fully exercised here
with a real MotionController over a fake connection + a scripted MockForceSensor.
Behaviors 2 & 3 (maintain_force / hold_compliant) only emit a command tuple (the
controller does the regulation, gated by FORCE_MODE_ENABLED), so we assert the
exact encoded command. No real sockets or robot.
"""

from __future__ import annotations

from unittest import mock

import pytest

from ur5e_control.config import RobotConfig
from ur5e_control.force.controller import ForceController
from ur5e_control.force.sensor import MockForceSensor
from ur5e_control.motion import MotionController


class FakeConn:
    """Records every sent command; no real socket; empty state stream."""

    def __init__(self):
        self.sent = []

    def send(self, msg):
        self.sent.append(msg)

    def latest_state(self):
        return ""


def _controller(sensor):
    conn = FakeConn()
    motion = MotionController(conn, RobotConfig())
    return ForceController(motion, sensor, RobotConfig()), conn


# --------------------------------------------------------------------------
# MotionController encode+send primitives
# --------------------------------------------------------------------------
def test_speed_l_encodes_cmd5_byte_exact():
    conn = FakeConn()
    MotionController(conn, RobotConfig()).speed_l([0, 0, -0.02, 0, 0, 0], accel=0.25, watchdog_t=2.0)
    assert conn.sent == ["(5, 0.0, 0.0, -0.02, 0.0, 0.0, 0.0, 0.25, 0.0, 2.0)"]


def test_force_push_encodes_cmd4():
    conn = FakeConn()
    MotionController(conn, RobotConfig()).force_push([0, 0, 1], 5.0, 0.05, 0.05)
    assert conn.sent[0].startswith("(4, 0.0, 0.0, 1.0, 5.0, 0.05, 0.05, ")


def test_impedance_encodes_cmd7():
    conn = FakeConn()
    MotionController(conn, RobotConfig()).impedance_hold([1, 1, 0], 300.0, 0.05, 0.05)
    assert conn.sent[0].startswith("(7, 1.0, 1.0, 0.0, 300.0, 0.05, 0.05, ")


def test_end_force_encodes_cmd6():
    conn = FakeConn()
    MotionController(conn, RobotConfig()).end_force()
    assert conn.sent[0].startswith("(6, ")


# --------------------------------------------------------------------------
# guarded_move (PC-side velocity, behavior 1)
# --------------------------------------------------------------------------
def test_guarded_move_stops_and_holds_at_threshold():
    # force along +Z climbs; the 3rd read crosses the 10 N threshold.
    sensor = MockForceSensor([[0, 0, 2, 0, 0, 0], [0, 0, 6, 0, 0, 0], [0, 0, 11, 0, 0, 0]])
    fc, conn = _controller(sensor)
    with mock.patch("ur5e_control.force.controller.time.sleep"):
        wrench = fc.guarded_move([0, 0, 1], speed=0.02, force_threshold_n=10.0, max_travel=0.1)
    assert conn.sent[0].startswith("(5, ")     # started with a speedl
    assert conn.sent[-1].startswith("(2, ")    # stopped (cmd=2) on contact -> hold
    assert wrench[2] == 11                      # returns the contact wrench


def test_guarded_move_raises_and_stops_if_no_contact():
    sensor = MockForceSensor([[0, 0, 1, 0, 0, 0]] * 1000)  # never reaches threshold
    fc, conn = _controller(sensor)
    with mock.patch("ur5e_control.force.controller.time.sleep"):
        with pytest.raises(RuntimeError):
            fc.guarded_move([0, 0, 1], speed=0.1, force_threshold_n=50.0, max_travel=0.05)
    assert conn.sent[-1].startswith("(2, ")    # still stops the robot on give-up


def test_guarded_move_rejects_bad_args():
    fc, _ = _controller(MockForceSensor([[0, 0, 0, 0, 0, 0]]))
    with pytest.raises(ValueError):
        fc.guarded_move([0, 0, 1], speed=0.0, force_threshold_n=10.0, max_travel=0.1)
    with pytest.raises(ValueError):
        fc.guarded_move([0, 0, 0], speed=0.02, force_threshold_n=10.0, max_travel=0.1)  # zero dir


# --------------------------------------------------------------------------
# maintain_force / hold_compliant (behaviors 2 & 3 — emit the command)
# --------------------------------------------------------------------------
def test_maintain_force_emits_cmd4():
    fc, conn = _controller(MockForceSensor([[0, 0, 0, 0, 0, 0]]))
    fc.maintain_force([0, 0, 1], target_n=8.0)
    assert conn.sent[0].startswith("(4, 0.0, 0.0, 1.0, 8.0, ")


def test_hold_compliant_emits_cmd7():
    fc, conn = _controller(MockForceSensor([[0, 0, 0, 0, 0, 0]]))
    fc.hold_compliant(compliant_axes=(1, 1, 0), stiffness=250.0)
    assert conn.sent[0].startswith("(7, 1.0, 1.0, 0.0, 250.0, ")


def test_end_force_emits_cmd6():
    fc, conn = _controller(MockForceSensor([[0, 0, 0, 0, 0, 0]]))
    fc.end_force()
    assert conn.sent[0].startswith("(6, ")
