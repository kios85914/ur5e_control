"""Tests for ur5e_control.motion — MotionController command encoding & moves.

The :class:`MotionController` turns high-level motion requests into the fixed
10-float ASCII command tuple documented for the PC -> daemon protocol and pushes
them through a :class:`~ur5e_control.connection.RobotConnection`. These tests use
a *mock* connection (no real sockets) so encoding, frame transforms, safety
checks, and the blocking convergence loop can be exercised deterministically.

Units/frames (matching the locked interface): meters and radians, UR base frame
on the wire. World-frame Cartesian inputs are converted via
:meth:`RobotConfig.world_to_ur` before encoding/sending.
"""

from unittest import mock

import pytest

from ur5e_control.config import RobotConfig
from ur5e_control.motion import MotionController
from ur5e_control.safety import (
    JointLimitViolation,
    SpeedViolation,
    WorkspaceViolation,
)


def _state_frame(pose):
    """Build a raw daemon state frame string whose TCP pose is ``pose``.

    Only the TCP pose group matters for convergence tests; the speed, joints and
    wrench groups are filled with zeros. Values are in meters/radians, UR base
    frame. The frame is terminated with '+' like the real wire format.
    """
    p = ",".join(str(v) for v in pose)
    zeros = "0,0,0,0,0,0"
    return f"p[{p}]_p[{zeros}]_[{zeros}]_p[{zeros}]+"


class FakeConnection:
    """Minimal stand-in for RobotConnection capturing sends and feeding states.

    ``sent`` records every string passed to :meth:`send`. :meth:`latest_state`
    returns successive frames from ``state_frames`` (repeating the last one once
    exhausted) so a blocking convergence loop can be driven deterministically.
    """

    def __init__(self, state_frames=None):
        self.sent = []
        self._frames = list(state_frames or [])
        self._idx = 0

    def send(self, msg):
        self.sent.append(msg)

    def latest_state(self):
        if not self._frames:
            return ""
        if self._idx < len(self._frames):
            frame = self._frames[self._idx]
            self._idx += 1
        else:
            frame = self._frames[-1]
        return frame


# ---------------------------------------------------------------------------
# encode_command — byte-exact tuple formatting
# ---------------------------------------------------------------------------
def test_encode_command_movel_byte_exact():
    """cmd=0 moveL encodes to the exact 10-field comma-space tuple string."""
    mc = MotionController(FakeConnection(), RobotConfig())
    s = mc.encode_command(0, [0.1, 0.2, 0.3, 0.0, -3.14, 0.0], 0.1, 0.1, 2.0)
    assert s == "(0, 0.1, 0.2, 0.3, 0.0, -3.14, 0.0, 0.1, 0.1, 2.0)"


def test_encode_command_movej_byte_exact():
    """cmd=1 moveJ encodes to the exact 10-field comma-space tuple string."""
    mc = MotionController(FakeConnection(), RobotConfig())
    s = mc.encode_command(1, [0.0, -1.57, 1.57, 0.0, 1.57, 0.0], 0.2, 0.15, 3.0)
    assert s == "(1, 0.0, -1.57, 1.57, 0.0, 1.57, 0.0, 0.2, 0.15, 3.0)"


def test_encode_command_stop_byte_exact():
    """cmd=2 stop encodes deceleration in a0 and zeros elsewhere in payload."""
    mc = MotionController(FakeConnection(), RobotConfig())
    s = mc.encode_command(2, [0.5, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.0)
    assert s == "(2, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)"


def test_encode_command_home_byte_exact():
    """cmd=3 home encodes an all-zero payload (daemon uses configured home)."""
    mc = MotionController(FakeConnection(), RobotConfig())
    s = mc.encode_command(3, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.1, 0.1, 2.0)
    assert s == "(3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.1, 2.0)"


def test_encode_command_rejects_bad_payload_length():
    """A payload that is not exactly 6 floats is rejected."""
    mc = MotionController(FakeConnection(), RobotConfig())
    with pytest.raises(ValueError):
        mc.encode_command(0, [0.1, 0.2, 0.3], 0.1, 0.1, 2.0)


# ---------------------------------------------------------------------------
# move_l — frame transform + encoding + send
# ---------------------------------------------------------------------------
def test_move_l_sends_movel_encoded_with_frame_transform():
    """move_l applies config.world_to_ur, then sends the moveL tuple."""
    conn = FakeConnection()
    cfg = RobotConfig()
    mc = MotionController(conn, cfg)

    world_pose = [-0.1, -0.4, 0.2, 0.0, -3.14, 0.0]
    mc.move_l(world_pose, speed=0.1, accel=0.1, blocking=False)

    # whatever the configured frame transform is, the sent pose is its output.
    expected = mc.encode_command(
        0, cfg.world_to_ur(world_pose), 0.1, 0.1, cfg.default_move_time
    )
    assert conn.sent == [expected]


def test_move_l_uses_config_defaults_for_speed_and_accel():
    """When speed/accel are omitted, config defaults are used in the tuple."""
    conn = FakeConnection()
    cfg = RobotConfig()
    mc = MotionController(conn, cfg)

    world_pose = [-0.1, -0.4, 0.2, 0.0, -3.14, 0.0]
    mc.move_l(world_pose, blocking=False)

    ur_pose = cfg.world_to_ur(world_pose)
    expected = mc.encode_command(
        0, ur_pose, cfg.default_accel, cfg.default_speed, cfg.default_move_time
    )
    assert conn.sent == [expected]


def test_move_l_relative_adds_to_current_world_pose():
    """relative=True offsets the current (world-frame) TCP pose by the delta."""
    # Current UR pose (in-workspace: y within 0.25..0.80) -> world pose via
    # config.ur_to_world (whatever the configured transform is).
    cfg = RobotConfig()
    current_ur = [0.0, -0.35, 0.2, 0.0, -3.14, 0.0]
    conn = FakeConnection(state_frames=[_state_frame(current_ur)])
    mc = MotionController(conn, cfg)

    delta = [0.05, 0.0, 0.01, 0.0, 0.0, 0.0]
    mc.move_l(delta, blocking=False, relative=True)

    current_world = cfg.ur_to_world(current_ur)
    target_world = [current_world[i] + delta[i] for i in range(6)]
    expected = mc.encode_command(
        0,
        cfg.world_to_ur(target_world),
        cfg.default_accel,
        cfg.default_speed,
        cfg.default_move_time,
    )
    assert conn.sent == [expected]


def test_move_l_runs_workspace_safety_before_send():
    """An out-of-workspace target raises and nothing is sent."""
    conn = FakeConnection()
    cfg = RobotConfig()
    mc = MotionController(conn, cfg)

    # z far above the 0.40 m ceiling -> WorkspaceViolation.
    bad_world = [0.0, -0.5, 5.0, 0.0, -3.14, 0.0]
    with pytest.raises(WorkspaceViolation):
        mc.move_l(bad_world, blocking=False)
    assert conn.sent == []


def test_move_l_rejects_nonpositive_speed():
    """A non-positive speed raises SpeedViolation and sends nothing."""
    conn = FakeConnection()
    mc = MotionController(conn, RobotConfig())
    with pytest.raises(SpeedViolation):
        mc.move_l([-0.1, -0.4, 0.2, 0.0, -3.14, 0.0], speed=0.0, blocking=False)
    assert conn.sent == []


def test_move_l_clamps_speed_to_max():
    """Speed above max_speed is clamped to max_speed in the encoded tuple."""
    conn = FakeConnection()
    cfg = RobotConfig()
    mc = MotionController(conn, cfg)

    world_pose = [-0.1, -0.4, 0.2, 0.0, -3.14, 0.0]
    mc.move_l(world_pose, speed=10.0, accel=0.1, blocking=False)

    expected = mc.encode_command(
        0, cfg.world_to_ur(world_pose), 0.1, cfg.max_speed, cfg.default_move_time
    )
    assert conn.sent == [expected]


# ---------------------------------------------------------------------------
# move_j — joint encoding + safety
# ---------------------------------------------------------------------------
def test_move_j_sends_movej_encoded():
    """move_j sends the moveJ tuple with joints unchanged (no frame transform)."""
    conn = FakeConnection()
    cfg = RobotConfig()
    mc = MotionController(conn, cfg)

    joints = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
    mc.move_j(joints, speed=0.2, accel=0.2, blocking=False)

    expected = mc.encode_command(1, joints, 0.2, 0.2, cfg.default_move_time)
    assert conn.sent == [expected]


def test_move_j_runs_joint_safety_before_send():
    """An out-of-limit joint raises JointLimitViolation and sends nothing."""
    conn = FakeConnection()
    cfg = RobotConfig()
    mc = MotionController(conn, cfg)

    bad = [100.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # > 2*pi limit
    with pytest.raises(JointLimitViolation):
        mc.move_j(bad, blocking=False)
    assert conn.sent == []


# ---------------------------------------------------------------------------
# stop / home
# ---------------------------------------------------------------------------
def test_stop_sends_cmd2():
    """stop() sends a cmd=2 tuple carrying a deceleration in a0."""
    conn = FakeConnection()
    mc = MotionController(conn, RobotConfig())
    mc.stop()
    assert len(conn.sent) == 1
    assert conn.sent[0].startswith("(2, ")


def test_home_sends_cmd3_blocking_false():
    """home(blocking=False) sends a single cmd=3 tuple with zero payload."""
    conn = FakeConnection()
    cfg = RobotConfig()
    mc = MotionController(conn, cfg)
    mc.home(blocking=False)
    expected = mc.encode_command(
        3, [0.0] * 6, cfg.default_accel, cfg.default_speed, cfg.default_move_time
    )
    assert conn.sent == [expected]


# ---------------------------------------------------------------------------
# blocking convergence loop
# ---------------------------------------------------------------------------
def test_move_l_blocking_exits_when_state_converges():
    """A blocking move returns once latest TCP pose is within convergence_tol."""
    cfg = RobotConfig()
    world_target = [-0.1, -0.4, 0.2, 0.0, -3.14, 0.0]
    ur_target = cfg.world_to_ur(world_target)

    # Far, then near (within 1e-3 of UR target on every axis).
    far = [v + 0.5 for v in ur_target]
    near = [v + 5e-4 for v in ur_target]
    conn = FakeConnection(state_frames=[_state_frame(far), _state_frame(near)])
    mc = MotionController(conn, cfg)

    with mock.patch("ur5e_control.motion.time.sleep"):
        mc.move_l(world_target, speed=0.1, accel=0.1, blocking=True)

    # Move tuple was sent and the loop consumed the converging frame.
    assert any(s.startswith("(0, ") for s in conn.sent)


def test_move_l_blocking_keeps_polling_until_within_tol():
    """The loop keeps polling while the pose stays outside convergence_tol."""
    cfg = RobotConfig()
    world_target = [-0.1, -0.4, 0.2, 0.0, -3.14, 0.0]
    ur_target = cfg.world_to_ur(world_target)

    far = _state_frame([v + 0.1 for v in ur_target])
    near = _state_frame([v + 1e-4 for v in ur_target])
    # Several far frames, then a converged one.
    conn = FakeConnection(state_frames=[far, far, far, near])
    mc = MotionController(conn, cfg)

    sleeps = []
    with mock.patch("ur5e_control.motion.time.sleep", side_effect=lambda *_: sleeps.append(1)):
        mc.move_l(world_target, blocking=True)

    # It polled (slept) at least once per non-converged frame before exiting.
    assert len(sleeps) >= 3


def test_blocking_ignores_empty_state_then_converges():
    """An empty latest_state ('' before any frame) is tolerated, not fatal."""
    cfg = RobotConfig()
    world_target = [-0.1, -0.4, 0.2, 0.0, -3.14, 0.0]
    ur_target = cfg.world_to_ur(world_target)

    near = _state_frame([v + 1e-4 for v in ur_target])
    conn = FakeConnection(state_frames=["", near])
    mc = MotionController(conn, cfg)

    with mock.patch("ur5e_control.motion.time.sleep"):
        mc.move_l(world_target, blocking=True)

    assert any(s.startswith("(0, ") for s in conn.sent)
