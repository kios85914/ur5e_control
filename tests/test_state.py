"""Tests for ur5e_control.state — RobotState dataclass and parse_state().

State stream format (daemon -> PC), units meters/radians, UR base frame:
    p[x,y,z,rx,ry,rz]_p[vx,vy,vz,wx,wy,wz]_[j0..j5]_p[fx,fy,fz,tx,ty,tz]+
"""

import math

import pytest

from ur5e_control.state import RobotState, parse_state


# A single well-formed frame in the documented format.
SAMPLE_FRAME = (
    "p[-0.06,0.25,0.115,0.0,-3.14,0.0]_"
    "p[0.01,0.02,0.03,0.04,0.05,0.06]_"
    "[0.1,0.2,0.3,0.4,0.5,0.6]_"
    "p[1.5,2.5,3.5,0.1,0.2,0.3]+"
)


def test_parse_state_returns_robotstate():
    state = parse_state(SAMPLE_FRAME)
    assert isinstance(state, RobotState)


def test_parse_state_field_order_and_types():
    """Every field must be a list[float] with the documented contents/order."""
    state = parse_state(SAMPLE_FRAME)

    assert state.tcp_pose == [-0.06, 0.25, 0.115, 0.0, -3.14, 0.0]
    assert state.tcp_speed == [0.01, 0.02, 0.03, 0.04, 0.05, 0.06]
    assert state.joints == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    assert state.wrench == [1.5, 2.5, 3.5, 0.1, 0.2, 0.3]

    for field in (state.tcp_pose, state.tcp_speed, state.joints, state.wrench):
        assert isinstance(field, list)
        assert len(field) == 6
        assert all(isinstance(v, float) for v in field)


def test_parse_state_sets_timestamp():
    import time

    before = time.time()
    state = parse_state(SAMPLE_FRAME)
    after = time.time()
    assert isinstance(state.timestamp, float)
    assert before <= state.timestamp <= after


def test_parse_state_uses_last_complete_frame_when_concatenated():
    """When several '+'-delimited frames arrive concatenated, use the LAST one."""
    first = (
        "p[1.0,1.0,1.0,1.0,1.0,1.0]_"
        "p[1.0,1.0,1.0,1.0,1.0,1.0]_"
        "[1.0,1.0,1.0,1.0,1.0,1.0]_"
        "p[1.0,1.0,1.0,1.0,1.0,1.0]+"
    )
    last = (
        "p[9.0,8.0,7.0,6.0,5.0,4.0]_"
        "p[0.0,0.0,0.0,0.0,0.0,0.0]_"
        "[0.9,0.8,0.7,0.6,0.5,0.4]_"
        "p[3.0,2.0,1.0,0.5,0.6,0.7]+"
    )
    state = parse_state(first + last)
    assert state.tcp_pose == [9.0, 8.0, 7.0, 6.0, 5.0, 4.0]
    assert state.joints == [0.9, 0.8, 0.7, 0.6, 0.5, 0.4]
    assert state.wrench == [3.0, 2.0, 1.0, 0.5, 0.6, 0.7]


def test_parse_state_ignores_trailing_incomplete_frame():
    """A trailing partial frame (no '+') is discarded; last COMPLETE frame wins."""
    partial = "p[0.0,0.0,0.0,0.0,0.0"  # truncated, no closing
    state = parse_state(SAMPLE_FRAME + partial)
    assert state.tcp_pose == [-0.06, 0.25, 0.115, 0.0, -3.14, 0.0]


def test_parse_state_negative_and_scientific_floats():
    frame = (
        "p[-1.0e-3,2.5,-3.14159,0.0,-0.0,1e2]_"
        "p[0.0,0.0,0.0,0.0,0.0,0.0]_"
        "[-6.283185,0.0,0.0,0.0,0.0,6.283185]_"
        "p[0.0,0.0,0.0,0.0,0.0,0.0]+"
    )
    state = parse_state(frame)
    assert math.isclose(state.tcp_pose[0], -1.0e-3)
    assert math.isclose(state.tcp_pose[5], 1e2)
    assert math.isclose(state.joints[0], -6.283185)


@pytest.mark.parametrize(
    "bad",
    [
        "",                       # empty
        "+",                      # no content
        "not a state at all",     # garbage
        "p[1,2,3]_p[1,2,3]_[1,2,3]_p[1,2,3]+",  # wrong element counts
        "p[1,2,3,4,5,6]_[1,2,3,4,5,6]_p[1,2,3,4,5,6]+",  # missing a group
        "p[1,2,3,4,5,x]_p[1,2,3,4,5,6]_[1,2,3,4,5,6]_p[1,2,3,4,5,6]+",  # non-float
        "p[1,2,3,4,5,6_p[1,2,3,4,5,6]_[1,2,3,4,5,6]_p[1,2,3,4,5,6]+",  # malformed bracket
    ],
)
def test_parse_state_malformed_raises_value_error(bad):
    with pytest.raises(ValueError):
        parse_state(bad)


def test_parse_state_non_string_raises_value_error():
    with pytest.raises(ValueError):
        parse_state(None)  # type: ignore[arg-type]
