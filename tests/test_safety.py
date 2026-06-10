"""Tests for ur5e_control.safety — limit validation for UR5e motion commands.

All poses are 6-lists [x, y, z, rx, ry, rz] in meters / radians, UR base frame.
Joints are 6-lists [j0..j5] in radians. Speeds are scalar in m/s (or rad/s for
joint moves); the cap is RobotConfig.max_speed.
"""

import pytest

from ur5e_control.config import RobotConfig
from ur5e_control.safety import (
    JointLimitViolation,
    SpeedViolation,
    WorkspaceViolation,
    check_joints,
    check_pose_in_workspace,
    clamp_speed,
)


@pytest.fixture
def config():
    """A default RobotConfig (locked defaults)."""
    return RobotConfig()


# ---------------------------------------------------------------------------
# check_pose_in_workspace
# ---------------------------------------------------------------------------

def test_pose_inside_bounds_passes(config):
    """A pose well inside every workspace limit returns True."""
    # Default limits: x (-0.40, 0.35), y (0.25, 0.80), z (0.0, 0.40).
    pose = [0.0, 0.5, 0.2, 0.0, -3.14, 0.0]
    assert check_pose_in_workspace(pose, config) is True


def test_pose_on_boundary_passes(config):
    """Poses exactly on the inclusive boundary are allowed."""
    xmin, xmax = config.workspace_limits["x"]
    ymin, ymax = config.workspace_limits["y"]
    zmin, zmax = config.workspace_limits["z"]
    assert check_pose_in_workspace([xmin, ymin, zmin, 0, 0, 0], config) is True
    assert check_pose_in_workspace([xmax, ymax, zmax, 0, 0, 0], config) is True


def test_pose_x_below_min_raises(config):
    xmin, _ = config.workspace_limits["x"]
    pose = [xmin - 0.01, 0.5, 0.2, 0.0, 0.0, 0.0]
    with pytest.raises(WorkspaceViolation):
        check_pose_in_workspace(pose, config)


def test_pose_x_above_max_raises(config):
    _, xmax = config.workspace_limits["x"]
    pose = [xmax + 0.01, 0.5, 0.2, 0.0, 0.0, 0.0]
    with pytest.raises(WorkspaceViolation):
        check_pose_in_workspace(pose, config)


def test_pose_y_below_min_raises(config):
    ymin, _ = config.workspace_limits["y"]
    pose = [0.0, ymin - 0.01, 0.2, 0.0, 0.0, 0.0]
    with pytest.raises(WorkspaceViolation):
        check_pose_in_workspace(pose, config)


def test_pose_y_above_max_raises(config):
    _, ymax = config.workspace_limits["y"]
    pose = [0.0, ymax + 0.01, 0.2, 0.0, 0.0, 0.0]
    with pytest.raises(WorkspaceViolation):
        check_pose_in_workspace(pose, config)


def test_pose_z_below_min_raises(config):
    zmin, _ = config.workspace_limits["z"]
    pose = [0.0, 0.5, zmin - 0.01, 0.0, 0.0, 0.0]
    with pytest.raises(WorkspaceViolation):
        check_pose_in_workspace(pose, config)


def test_pose_z_above_max_raises(config):
    _, zmax = config.workspace_limits["z"]
    pose = [0.0, 0.5, zmax + 0.01, 0.0, 0.0, 0.0]
    with pytest.raises(WorkspaceViolation):
        check_pose_in_workspace(pose, config)


def test_pose_wrong_length_raises(config):
    """A pose that is not a 6-list is rejected."""
    with pytest.raises(WorkspaceViolation):
        check_pose_in_workspace([0.0, 0.5, 0.2], config)


def test_workspace_violation_message_names_axis(config):
    """The raised error mentions which axis was violated (x here)."""
    _, xmax = config.workspace_limits["x"]
    with pytest.raises(WorkspaceViolation) as exc:
        check_pose_in_workspace([xmax + 1.0, 0.5, 0.2, 0, 0, 0], config)
    assert "x" in str(exc.value).lower()


# ---------------------------------------------------------------------------
# check_joints
# ---------------------------------------------------------------------------

def test_joints_inside_bounds_passes(config):
    """Six joints within the configured limits return True."""
    joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert check_joints(joints, config) is True


def test_joints_on_boundary_passes(config):
    """Joints exactly on the inclusive boundary are allowed."""
    boundary = [lo for (lo, _hi) in config.joint_limits]
    assert check_joints(boundary, config) is True
    boundary_hi = [hi for (_lo, hi) in config.joint_limits]
    assert check_joints(boundary_hi, config) is True


def test_joints_wrong_count_too_few_raises(config):
    with pytest.raises(JointLimitViolation):
        check_joints([0.0, 0.0, 0.0], config)


def test_joints_wrong_count_too_many_raises(config):
    with pytest.raises(JointLimitViolation):
        check_joints([0.0] * 7, config)


def test_joints_out_of_range_below_raises(config):
    lo, _hi = config.joint_limits[2]
    joints = [0.0, 0.0, lo - 0.5, 0.0, 0.0, 0.0]
    with pytest.raises(JointLimitViolation):
        check_joints(joints, config)


def test_joints_out_of_range_above_raises(config):
    _lo, hi = config.joint_limits[4]
    joints = [0.0, 0.0, 0.0, 0.0, hi + 0.5, 0.0]
    with pytest.raises(JointLimitViolation):
        check_joints(joints, config)


def test_joint_violation_message_names_index(config):
    _lo, hi = config.joint_limits[3]
    with pytest.raises(JointLimitViolation) as exc:
        check_joints([0.0, 0.0, 0.0, hi + 10.0, 0.0, 0.0], config)
    assert "3" in str(exc.value)


# ---------------------------------------------------------------------------
# clamp_speed
# ---------------------------------------------------------------------------

def test_clamp_speed_below_cap_unchanged(config):
    """A speed under the cap is returned as-is."""
    assert clamp_speed(0.05, config) == pytest.approx(0.05)


def test_clamp_speed_equal_cap_unchanged(config):
    assert clamp_speed(config.max_speed, config) == pytest.approx(config.max_speed)


def test_clamp_speed_above_cap_is_capped(config):
    """A speed exceeding max_speed is clamped down to max_speed."""
    assert clamp_speed(config.max_speed + 1.0, config) == pytest.approx(config.max_speed)


def test_clamp_speed_negative_raises(config):
    """A negative speed is invalid and raises SpeedViolation."""
    with pytest.raises(SpeedViolation):
        clamp_speed(-0.1, config)


def test_clamp_speed_zero_raises(config):
    """Zero speed would never converge; rejected."""
    with pytest.raises(SpeedViolation):
        clamp_speed(0.0, config)


# ---------------------------------------------------------------------------
# exception hierarchy
# ---------------------------------------------------------------------------

def test_exceptions_are_distinct_types():
    assert issubclass(WorkspaceViolation, Exception)
    assert issubclass(SpeedViolation, Exception)
    assert issubclass(JointLimitViolation, Exception)
    assert WorkspaceViolation is not SpeedViolation
    assert SpeedViolation is not JointLimitViolation
