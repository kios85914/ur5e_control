"""Tests for ur5e_control.force.controller — ForceController approach-until-force.

The :class:`ForceController` implements a pure, mock-testable control law:
drive the TCP along a direction with small relative moves while monitoring a
:class:`~ur5e_control.force.sensor.ForceSensor`, and the instant the measured
force magnitude **along that direction** reaches the target, stop approaching
and transition to a held URScript force-mode command (cmd=4).

Units & frames (matching the locked interface): meters and radians, forces in
newtons, expressed in the UR base frame.

These tests use a scripted :class:`~ur5e_control.force.sensor.MockForceSensor`
(below target, below target, then at/above target) and a *mock* motion
controller so the law is exercised deterministically with no real robot:

* it stops/holds at the contact step (no extra approach step afterwards),
* it does not overshoot past the scripted contact step, and
* it emits a cmd=4 force payload with the correct direction/target encoding
  (a0..a2 = unit direction, a3 = target_n, a4 = stiffness, a5 = max_travel).
"""

import math

import pytest

from ur5e_control.config import RobotConfig
from ur5e_control.force.controller import ForceController
from ur5e_control.force.sensor import MockForceSensor


# Force-mode opcode in the PC -> daemon protocol (cmd=4 force).
_CMD_FORCE = 4


def _z6(*vals):
    """Return a 6-vector padded with zeros, for terse wrench literals."""
    out = list(vals) + [0.0] * (6 - len(vals))
    return [float(v) for v in out]


class MockMotion:
    """Mock motion controller capturing approach steps and force-mode sends.

    Mirrors the small slice of :class:`~ur5e_control.motion.MotionController`
    the controller relies on:

    * :meth:`move_l` — records each commanded (pose, relative) approach step.
    * :meth:`encode_command` — same static encoding contract as the real
      controller (so the emitted force tuple is byte-checkable).
    * :meth:`send_command` — records the raw command string handed to the
      transport (the force-mode hold tuple).

    Attributes:
        moves: list of dicts ``{"pose", "relative", "blocking"}`` per move_l call.
        sent: list of raw command strings passed to :meth:`send_command`.
        encoded: list of ``(cmd, payload, accel, vel, time)`` tuples encoded.
    """

    def __init__(self):
        self.moves = []
        self.sent = []
        self.encoded = []

    def move_l(self, pose, speed=None, accel=None, blocking=True, relative=False):
        self.moves.append(
            {
                "pose": list(pose),
                "speed": speed,
                "accel": accel,
                "blocking": blocking,
                "relative": relative,
            }
        )

    @staticmethod
    def encode_command(cmd, payload, accel, vel, time):
        payload = list(payload)
        fields = [str(int(cmd))]
        fields.extend(str(float(v)) for v in payload)
        fields.append(str(float(accel)))
        fields.append(str(float(vel)))
        fields.append(str(float(time)))
        return "(" + ", ".join(fields) + ")"

    def send_command(self, msg):
        self.sent.append(msg)


def _parse_tuple(msg):
    """Parse a "(cmd, a0..a5, accel, vel, time)" string into floats."""
    inner = msg.strip()[1:-1]
    return [float(tok) for tok in inner.split(",")]


# --------------------------------------------------------------------------- #
# Contact detection: stop & hold at the contact step, no overshoot.
# --------------------------------------------------------------------------- #
def test_holds_on_first_step_that_reaches_target():
    """Force reached on the 3rd read -> step on each below-target read, then hold.

    Semantics: each iteration reads first; a below-target read produces one
    approach step, the contact read produces *no* step (stop/hold exactly at
    contact, no overshoot). Two below-target reads -> two approach steps.
    """
    motion = MockMotion()
    # Approach along +z. Below, below, then at/above target (5 N).
    sensor = MockForceSensor(
        [
            _z6(0.0, 0.0, 1.0),  # below -> step
            _z6(0.0, 0.0, 3.0),  # below -> step
            _z6(0.0, 0.0, 6.0),  # >= target -> contact, no step
            _z6(0.0, 0.0, 99.0),  # must never be read
        ]
    )
    ctrl = ForceController(motion, sensor, RobotConfig())

    ctrl.approach_until_force(
        direction=[0.0, 0.0, 1.0], target_n=5.0, stiffness=0.05, max_travel=0.1
    )

    # Two below-target reads -> two approach steps; the contact step adds none.
    assert len(motion.moves) == 2
    # Exactly one force-mode hold command emitted.
    assert len(motion.sent) == 1


def test_no_overshoot_does_not_step_after_contact():
    """No approach step is issued after the contact step is detected."""
    motion = MockMotion()
    sensor = MockForceSensor(
        [
            _z6(0.0, 0.0, 10.0),  # already at/above target on first read
            _z6(0.0, 0.0, 50.0),  # must not be consumed
        ]
    )
    ctrl = ForceController(motion, sensor, RobotConfig())

    ctrl.approach_until_force(
        direction=[0.0, 0.0, 1.0], target_n=5.0, stiffness=0.05, max_travel=0.1
    )

    # Contact on the very first read -> zero approach steps before holding.
    assert motion.moves == []
    assert len(motion.sent) == 1


def test_approach_steps_are_relative_along_direction():
    """Each approach step is a relative move pointing along the unit direction."""
    motion = MockMotion()
    sensor = MockForceSensor(
        [
            _z6(0.0, 0.0, 1.0),
            _z6(0.0, 0.0, 6.0),  # contact on 2nd read
        ]
    )
    ctrl = ForceController(motion, sensor, RobotConfig())

    ctrl.approach_until_force(
        direction=[0.0, 0.0, 2.0],  # non-unit -> must be normalized
        target_n=5.0,
        stiffness=0.05,
        max_travel=0.1,
    )

    assert len(motion.moves) == 1
    step = motion.moves[0]
    assert step["relative"] is True
    # Step is along +z only (normalized direction), x and y unchanged.
    assert step["pose"][0] == pytest.approx(0.0)
    assert step["pose"][1] == pytest.approx(0.0)
    assert step["pose"][2] > 0.0
    # Rotation components are never disturbed by an approach step.
    assert step["pose"][3:] == pytest.approx([0.0, 0.0, 0.0])


# --------------------------------------------------------------------------- #
# Force-mode (cmd=4) payload encoding.
# --------------------------------------------------------------------------- #
def test_emits_cmd4_force_payload_with_correct_encoding():
    """The hold command is cmd=4 with a0..a2=unit dir, a3=target, a4=stiff, a5=travel."""
    motion = MockMotion()
    sensor = MockForceSensor([_z6(0.0, 0.0, 7.0)])  # immediate contact
    ctrl = ForceController(motion, sensor, RobotConfig())

    ctrl.approach_until_force(
        direction=[0.0, 0.0, 3.0],  # non-unit; expect normalized to [0,0,1]
        target_n=5.0,
        stiffness=0.08,
        max_travel=0.12,
    )

    assert len(motion.sent) == 1
    fields = _parse_tuple(motion.sent[0])
    assert int(fields[0]) == _CMD_FORCE
    # a0..a2 = unit direction vector.
    assert fields[1] == pytest.approx(0.0)
    assert fields[2] == pytest.approx(0.0)
    assert fields[3] == pytest.approx(1.0)
    # a3 = target force (N), a4 = stiffness, a5 = max travel (m).
    assert fields[4] == pytest.approx(5.0)
    assert fields[5] == pytest.approx(0.08)
    assert fields[6] == pytest.approx(0.12)


def test_force_payload_normalizes_arbitrary_direction():
    """An arbitrary direction is encoded as a unit vector in a0..a2."""
    motion = MockMotion()
    sensor = MockForceSensor([_z6(3.0, 0.0, 4.0)])  # |proj| along dir = 5 N
    ctrl = ForceController(motion, sensor, RobotConfig())

    ctrl.approach_until_force(
        direction=[3.0, 0.0, 4.0],  # magnitude 5 -> unit [0.6, 0, 0.8]
        target_n=5.0,
        stiffness=0.05,
        max_travel=0.1,
    )

    fields = _parse_tuple(motion.sent[0])
    assert int(fields[0]) == _CMD_FORCE
    assert fields[1] == pytest.approx(0.6)
    assert fields[2] == pytest.approx(0.0)
    assert fields[3] == pytest.approx(0.8)
    # The encoded unit vector has unit norm.
    norm = math.sqrt(fields[1] ** 2 + fields[2] ** 2 + fields[3] ** 2)
    assert norm == pytest.approx(1.0)


# --------------------------------------------------------------------------- #
# Force is measured ALONG the direction (projection), not raw magnitude.
# --------------------------------------------------------------------------- #
def test_uses_force_projected_onto_direction():
    """Force perpendicular to the approach direction does not trigger contact."""
    motion = MockMotion()
    sensor = MockForceSensor(
        [
            _z6(100.0, 0.0, 0.0),  # huge force, but perpendicular to +z -> no contact
            _z6(0.0, 100.0, 0.0),  # again perpendicular -> no contact
            _z6(0.0, 0.0, 6.0),  # along +z, >= target -> contact
        ]
    )
    ctrl = ForceController(motion, sensor, RobotConfig())

    ctrl.approach_until_force(
        direction=[0.0, 0.0, 1.0], target_n=5.0, stiffness=0.05, max_travel=0.1
    )

    # Two perpendicular (below-target along +z) reads -> two approach steps; the
    # third read is contact and adds no step.
    assert len(motion.moves) == 2
    assert len(motion.sent) == 1


# --------------------------------------------------------------------------- #
# Safety: max travel guard prevents an unbounded approach.
# --------------------------------------------------------------------------- #
def test_aborts_when_max_travel_exceeded_without_contact():
    """If contact is never reached within max_travel, the approach aborts (no hold)."""
    motion = MockMotion()
    # Always below target; with a tiny max_travel the loop must give up.
    sensor = MockForceSensor([_z6(0.0, 0.0, 1.0)] * 1000)
    ctrl = ForceController(motion, sensor, RobotConfig())

    with pytest.raises(RuntimeError):
        ctrl.approach_until_force(
            direction=[0.0, 0.0, 1.0],
            target_n=5.0,
            stiffness=0.05,
            max_travel=0.02,  # small budget relative to the step size
        )

    # No force-mode hold should have been emitted on abort.
    assert motion.sent == []


# --------------------------------------------------------------------------- #
# Input validation.
# --------------------------------------------------------------------------- #
def test_rejects_zero_direction():
    motion = MockMotion()
    sensor = MockForceSensor([_z6(0.0, 0.0, 6.0)])
    ctrl = ForceController(motion, sensor, RobotConfig())
    with pytest.raises(ValueError):
        ctrl.approach_until_force(
            direction=[0.0, 0.0, 0.0], target_n=5.0, stiffness=0.05, max_travel=0.1
        )


def test_rejects_wrong_length_direction():
    motion = MockMotion()
    sensor = MockForceSensor([_z6(0.0, 0.0, 6.0)])
    ctrl = ForceController(motion, sensor, RobotConfig())
    with pytest.raises(ValueError):
        ctrl.approach_until_force(
            direction=[0.0, 1.0], target_n=5.0, stiffness=0.05, max_travel=0.1
        )


def test_rejects_nonpositive_target_force():
    motion = MockMotion()
    sensor = MockForceSensor([_z6(0.0, 0.0, 6.0)])
    ctrl = ForceController(motion, sensor, RobotConfig())
    with pytest.raises(ValueError):
        ctrl.approach_until_force(
            direction=[0.0, 0.0, 1.0], target_n=0.0, stiffness=0.05, max_travel=0.1
        )


def test_rejects_nonpositive_max_travel():
    motion = MockMotion()
    sensor = MockForceSensor([_z6(0.0, 0.0, 6.0)])
    ctrl = ForceController(motion, sensor, RobotConfig())
    with pytest.raises(ValueError):
        ctrl.approach_until_force(
            direction=[0.0, 0.0, 1.0], target_n=5.0, stiffness=0.05, max_travel=0.0
        )
