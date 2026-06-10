"""Tests for ur5e_control.force.sensor — ForceSensor ABC + backends.

Wrenches are 6-element lists ``[fx, fy, fz, tx, ty, tz]`` in N and Nm,
expressed in the UR base frame (see RobotState). These tests cover:

* ``MockForceSensor`` replays a scripted sequence of wrenches in order and
  supports ``zero()``.
* ``RobotiqFT300`` extracts the wrench from the latest ``RobotState`` returned
  by an injected ``state_provider`` callable (the swappable read path).
"""

from abc import ABC

import pytest

from ur5e_control.force.sensor import ForceSensor, MockForceSensor, RobotiqFT300
from ur5e_control.state import RobotState


def _make_state(wrench):
    """Build a RobotState carrying the given wrench (other fields are dummy)."""
    return RobotState(
        tcp_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        tcp_speed=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        joints=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        wrench=list(wrench),
        timestamp=0.0,
    )


# --------------------------------------------------------------------------- #
# ForceSensor ABC
# --------------------------------------------------------------------------- #
def test_force_sensor_is_abstract():
    """ForceSensor is an ABC and cannot be instantiated directly."""
    assert issubclass(ForceSensor, ABC)
    with pytest.raises(TypeError):
        ForceSensor()  # type: ignore[abstract]


def test_force_sensor_subclasses_are_force_sensors():
    """Both concrete backends register as ForceSensor instances."""
    assert issubclass(MockForceSensor, ForceSensor)
    assert issubclass(RobotiqFT300, ForceSensor)


# --------------------------------------------------------------------------- #
# MockForceSensor
# --------------------------------------------------------------------------- #
def test_mock_returns_scripted_sequence_in_order():
    seq = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [1.0, 2.0, 3.0, 0.1, 0.2, 0.3],
        [-4.0, 5.0, -6.0, 0.4, 0.5, 0.6],
    ]
    sensor = MockForceSensor(seq)
    assert sensor.read() == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert sensor.read() == [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]
    assert sensor.read() == [-4.0, 5.0, -6.0, 0.4, 0.5, 0.6]


def test_mock_read_returns_six_floats():
    sensor = MockForceSensor([[1, 2, 3, 4, 5, 6]])
    wrench = sensor.read()
    assert isinstance(wrench, list)
    assert len(wrench) == 6
    assert all(isinstance(v, float) for v in wrench)


def test_mock_read_returns_independent_copy():
    """Mutating a returned wrench must not corrupt the stored sequence."""
    seq = [[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]]
    sensor = MockForceSensor(seq)
    first = sensor.read()
    first[0] = 999.0
    # Re-create and read again from the same data: stored sequence untouched.
    assert seq[0] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_mock_exhausted_sequence_raises():
    sensor = MockForceSensor([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
    sensor.read()
    with pytest.raises(IndexError):
        sensor.read()


def test_mock_zero_resets_to_start_of_sequence():
    seq = [
        [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        [2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
    ]
    sensor = MockForceSensor(seq)
    assert sensor.read() == [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    sensor.zero()
    # After zero() the replay restarts from the beginning.
    assert sensor.read() == [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    assert sensor.read() == [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]


def test_mock_rejects_wrong_length_wrench():
    with pytest.raises(ValueError):
        MockForceSensor([[1.0, 2.0, 3.0]])


# --------------------------------------------------------------------------- #
# RobotiqFT300 (streamed-force backend)
# --------------------------------------------------------------------------- #
def test_ft300_read_returns_wrench_from_state_provider():
    expected = [10.0, -20.0, 30.0, 0.7, -0.8, 0.9]
    sensor = RobotiqFT300(lambda: _make_state(expected))
    assert sensor.read() == expected


def test_ft300_read_reflects_latest_state():
    """Each read() pulls the *current* state from the provider."""
    box = {"wrench": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}
    sensor = RobotiqFT300(lambda: _make_state(box["wrench"]))
    assert sensor.read() == [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    box["wrench"] = [5.0, 6.0, 7.0, 0.1, 0.2, 0.3]
    assert sensor.read() == [5.0, 6.0, 7.0, 0.1, 0.2, 0.3]


def test_ft300_read_returns_six_floats():
    sensor = RobotiqFT300(lambda: _make_state([1, 2, 3, 4, 5, 6]))
    wrench = sensor.read()
    assert isinstance(wrench, list)
    assert len(wrench) == 6
    assert all(isinstance(v, float) for v in wrench)


def test_ft300_read_returns_independent_copy():
    """A returned wrench must not alias the RobotState's wrench list."""
    state = _make_state([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    sensor = RobotiqFT300(lambda: state)
    wrench = sensor.read()
    wrench[0] = 999.0
    assert state.wrench[0] == 1.0


def test_ft300_zero_is_callable():
    """zero() exists and does not raise for the streamed backend."""
    sensor = RobotiqFT300(lambda: _make_state([0.0] * 6))
    sensor.zero()  # should not raise


def test_ft300_requires_callable_state_provider():
    with pytest.raises(TypeError):
        RobotiqFT300(state_provider=None)  # type: ignore[arg-type]
