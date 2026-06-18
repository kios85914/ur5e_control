"""Tests for ur5e_control.config — RobotConfig dataclass and frame transforms.

All poses are [x, y, z, rx, ry, rz] with positions in meters and rotations in
radians. The default frame convention negates only x and y between the world
frame and the UR base frame; z and rotations are unchanged.
"""

import math

import pytest

from ur5e_control.config import RobotConfig


def test_world_to_ur_default_is_identity():
    """Default world<->UR transform is identity (world == UR base frame)."""
    cfg = RobotConfig()
    pose = [0.1, 0.3, 0.2, 0.4, -3.0, 0.5]  # meters / radians
    assert cfg.world_to_ur(pose) == [0.1, 0.3, 0.2, 0.4, -3.0, 0.5]
    assert cfg.ur_to_world(pose) == [0.1, 0.3, 0.2, 0.4, -3.0, 0.5]


def test_world_to_ur_round_trips_to_identity():
    """world_to_ur followed by ur_to_world returns the original pose."""
    cfg = RobotConfig()
    pose = [0.12, 0.45, 0.18, 0.0, -3.14, 0.07]  # meters / radians, world frame
    round_tripped = cfg.ur_to_world(cfg.world_to_ur(pose))
    assert round_tripped == pytest.approx(pose)

    # And the reverse order also round-trips to identity.
    ur_pose = [-0.06, -0.25, 0.115, 0.0, -3.14, 0.0]  # meters / radians, UR base frame
    assert cfg.world_to_ur(cfg.ur_to_world(ur_pose)) == pytest.approx(ur_pose)


def test_world_to_ur_does_not_mutate_input():
    """Transforms must not mutate the caller's list."""
    cfg = RobotConfig()
    pose = [0.1, 0.3, 0.2, 0.4, -3.0, 0.5]
    original = list(pose)
    cfg.world_to_ur(pose)
    assert pose == original


def test_defaults_match_locked_values():
    """All RobotConfig defaults equal the locked interface values."""
    cfg = RobotConfig()
    assert cfg.controller_ip == "192.168.0.137"
    assert cfg.script_port == 30001
    assert cfg.pc_host == "192.168.0.120"
    assert cfg.state_port == 30002
    assert cfg.default_speed == 0.1
    assert cfg.default_accel == 0.1
    # 0.0 so the URScript `time` arg does not override `speed` (see config docs).
    assert cfg.default_move_time == 0.0
    assert cfg.convergence_tol == 1e-3
    assert cfg.max_speed == 0.25
    assert cfg.home_pose == [0.0, -0.35, 0.25, 0.0, -3.14, 0.0]
    # Force-mode branches disabled by default (armed via render_daemon).
    assert cfg.force_mode_enabled is False
    # force_mode stability tuning defaults. gain_scaling is None by default
    # because force_mode_set_gain_scaling does not exist on CB-series ("G3").
    assert cfg.force_mode_damping == 0.2
    assert cfg.force_mode_gain_scaling is None
    # Real FT 300/FT 300-S reader (URCap port 63351) off by default.
    assert cfg.ft300_enabled is False
    assert cfg.ft300_port == 63351


def test_home_pose_has_length_6():
    """home_pose is a 6-element pose [x, y, z, rx, ry, rz]."""
    cfg = RobotConfig()
    assert len(cfg.home_pose) == 6


def test_workspace_limits_has_xyz_tuples():
    """workspace_limits has x/y/z keys each mapping to a (min, max) tuple."""
    cfg = RobotConfig()
    limits = cfg.workspace_limits
    assert set(limits.keys()) == {"x", "y", "z"}
    assert limits["x"] == (-0.40, 0.40)
    assert limits["y"] == (-0.565, -0.265)
    assert limits["z"] == (-0.10, 0.40)
    for axis in ("x", "y", "z"):
        lo, hi = limits[axis]
        assert lo < hi


def test_joint_limits_six_min_max_tuples():
    """joint_limits is a tuple of 6 (min, max) pairs in radians."""
    cfg = RobotConfig()
    assert len(cfg.joint_limits) == 6
    for lo, hi in cfg.joint_limits:
        assert lo == pytest.approx(-6.283185)
        assert hi == pytest.approx(6.283185)
        assert lo < hi


def test_mutable_defaults_are_independent_between_instances():
    """Mutable defaults (lists/dicts) must not be shared across instances."""
    a = RobotConfig()
    b = RobotConfig()
    assert a.home_pose is not b.home_pose
    assert a.workspace_limits is not b.workspace_limits
    a.home_pose[0] = 999.0
    a.workspace_limits["x"] = (0.0, 0.0)
    assert b.home_pose[0] == 0.0
    assert b.workspace_limits["x"] == (-0.40, 0.40)


def test_full_round_trip_random_like_values():
    """Round-trip identity holds for a spread of values across all axes."""
    cfg = RobotConfig()
    poses = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [-0.40, 0.25, 0.40, math.pi, -math.pi, math.pi / 2],
        [0.35, 0.80, 0.0, -1.23, 2.34, -0.01],
    ]
    for pose in poses:
        assert cfg.ur_to_world(cfg.world_to_ur(pose)) == pytest.approx(pose)
