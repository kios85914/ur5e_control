"""Runnable examples for the ur5e_control library.

Each example is a small, self-contained script with a ``main()`` entry point and
a ``python -m`` / direct-run guard, so it can be executed straight from a clone:

* :mod:`ur5e_control.examples.move_example` — drive
  :class:`~ur5e_control.robot.UR5eRobot` (move, read state, home) in a safe
  **dry-run** preview that opens no sockets and commands no real robot.
* :mod:`ur5e_control.examples.force_control_example` — demonstrate
  :meth:`~ur5e_control.force.controller.ForceController.approach_until_force`
  against a mock sensor/motion (a real Robotiq FT 300 is required for live use).

All poses are ``[x, y, z, rx, ry, rz]`` in **meters / radians**; world-frame
Cartesian inputs are converted to the UR base frame inside the library.
"""
