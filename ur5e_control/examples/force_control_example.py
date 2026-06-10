"""Example: force-guided approach with :class:`ForceController` (mock hardware).

This demonstrates
:meth:`~ur5e_control.force.controller.ForceController.approach_until_force`:
drive the TCP in small relative linear steps along a commanded direction while
watching the force/torque sensor, and the instant the measured force **projected
onto that direction** reaches the target, stop approaching and transition to a
held URScript force-mode command (``cmd=4``).

.. warning::

   **A real Robotiq FT 300 is required for live use.** This example runs entirely
   against a scripted :class:`~ur5e_control.force.sensor.MockForceSensor` and a
   *mock* motion controller — it touches no hardware and opens no sockets. The
   step size, the force threshold, the ``stiffness`` and the ``max_travel``
   budget here are illustrative placeholders; they **must be re-tuned on the real
   robot** (with the FT 300 mounted) before any contact task is trusted.

Units & frames (matching the library): the approach ``direction`` is a 3-vector
in the **UR base frame**; ``target_n`` is in **newtons (N)**; ``step`` and
``max_travel`` are in **meters (m)**; wrenches are ``[fx, fy, fz, tx, ty, tz]``
(N, Nm), UR base frame.

Running it
----------
::

    python -m ur5e_control.examples.force_control_example
    python ur5e_control/examples/force_control_example.py
"""

from __future__ import annotations

import sys
from pathlib import Path

# Allow running this file directly
# (``python ur5e_control/examples/force_control_example.py``): put the project
# root (three levels up: <root>/ur5e_control/examples/<file>) on sys.path so
# ``import ur5e_control`` resolves without an editable install.
_PROJECT_ROOT = str(Path(__file__).resolve().parents[2])
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from ur5e_control.config import RobotConfig
from ur5e_control.force.controller import ForceController
from ur5e_control.force.sensor import MockForceSensor

# Approach straight down along -Z in the UR base frame (toward a surface below).
_APPROACH_DIRECTION = [0.0, 0.0, -1.0]

# Demo contact target (newtons), step (m), and travel budget (m). PLACEHOLDERS —
# re-tune on the real robot with the FT 300 mounted (see the module warning).
_TARGET_FORCE_N = 5.0
_STEP_M = 0.005
_STIFFNESS = 0.05
_MAX_TRAVEL_M = 0.05

# Scripted wrenches [fx, fy, fz, tx, ty, tz] (N, Nm), UR base frame. Force grows
# along -Z over a few reads; the projection onto the approach direction
# (-fz here) crosses _TARGET_FORCE_N on the final entry, modelling contact.
_SCRIPTED_WRENCHES = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   # free space, no contact
    [0.0, 0.0, -1.5, 0.0, 0.0, 0.0],  # light touch
    [0.0, 0.0, -3.0, 0.0, 0.0, 0.0],  # pressing in
    [0.0, 0.0, -6.0, 0.0, 0.0, 0.0],  # >= 5 N along -Z -> contact reached
]


class _LoggingMockMotion:
    """Minimal mock motion controller that logs the calls ForceController makes.

    :class:`~ur5e_control.force.controller.ForceController` depends only on a
    ``motion`` object exposing ``move_l`` (relative approach steps),
    ``encode_command`` (static protocol encoding), and a send path
    (``send_command`` is preferred). This stand-in records and prints each, so
    the example shows exactly what would go to a real daemon without sending it
    anywhere.

    Attributes:
        steps: list of relative approach delta poses (meters/radians) issued.
        sent: list of raw force-mode command strings that would be transmitted.
    """

    def __init__(self) -> None:
        self.steps: list[list[float]] = []
        self.sent: list[str] = []

    def move_l(
        self,
        pose,
        speed=None,
        accel=None,
        blocking: bool = True,
        relative: bool = False,
    ) -> None:
        """Record (and print) one relative approach step in meters/radians."""
        delta = list(pose)
        self.steps.append(delta)
        print(f"   approach step (relative={relative}): {delta} m,rad")

    @staticmethod
    def encode_command(cmd, payload, accel, vel, time) -> str:
        """Encode a command tuple exactly like the real MotionController.

        Mirrors the locked PC -> daemon protocol:
        ``"(cmd, a0..a5, accel, vel, time)"`` (forces in N, distances in m).
        """
        fields = [str(int(cmd))]
        fields.extend(str(float(v)) for v in payload)
        fields.append(str(float(accel)))
        fields.append(str(float(vel)))
        fields.append(str(float(time)))
        return "(" + ", ".join(fields) + ")"

    def send_command(self, msg: str) -> None:
        """Record (and print) the raw force-mode command string."""
        self.sent.append(msg)
        print(f"   would send: {msg}")


def main() -> None:
    """Run the approach-until-force demo against the mock sensor/motion.

    Builds a :class:`ForceController` over a :class:`MockForceSensor` (scripted to
    reach contact) and a logging mock motion controller, then calls
    :meth:`~ur5e_control.force.controller.ForceController.approach_until_force`
    along ``-Z`` until the projected force reaches ``_TARGET_FORCE_N`` newtons,
    at which point a held ``cmd=4`` force-mode command is emitted.

    Notes:
        No sockets are opened and no hardware is touched. A real Robotiq FT 300
        and on-robot tuning are required for live use (see the module warning).
    """
    print("== UR5e force-control example — MOCK sensor/motion (no hardware) ==")
    print("WARNING: a real Robotiq FT 300 is required for live use; the numbers")
    print("         below are placeholders and must be re-tuned on the robot.\n")

    config = RobotConfig()
    sensor = MockForceSensor(_SCRIPTED_WRENCHES)
    motion = _LoggingMockMotion()
    controller = ForceController(motion, sensor, config)

    print(
        f"approach_until_force: dir={_APPROACH_DIRECTION} (UR base frame), "
        f"target={_TARGET_FORCE_N} N, step={_STEP_M} m, "
        f"max_travel={_MAX_TRAVEL_M} m, stiffness={_STIFFNESS}"
    )
    controller.approach_until_force(
        direction=_APPROACH_DIRECTION,
        target_n=_TARGET_FORCE_N,
        stiffness=_STIFFNESS,
        max_travel=_MAX_TRAVEL_M,
        step=_STEP_M,
    )

    print(
        f"\nContact reached after {len(motion.steps)} approach step(s); "
        f"{len(motion.sent)} force-mode command(s) emitted."
    )
    print("Done.")


if __name__ == "__main__":
    main()
