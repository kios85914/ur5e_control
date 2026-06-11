"""Local web control panel for ur5e_control.

A small standard-library HTTP server (:mod:`ur5e_control.gui.server`) bridges a
browser UI to a single :class:`~ur5e_control.robot.UR5eRobot`: set the IP, check
the connection, watch live state, and jog the TCP / joints — a one-screen test
harness for every library feature.

Run it with::

    python -m ur5e_control.gui          # then open http://127.0.0.1:8080

It binds to localhost only and starts in **dry-run** by default (no robot needed
to exercise the whole UI). No third-party dependencies.
"""
