"""Tests for ur5e_control.examples — the runnable example scripts.

The examples in :mod:`ur5e_control.examples` are meant to be run by a human with
no robot attached, as a *safe preview* of the library's API:

* ``move_example`` drives :class:`~ur5e_control.robot.UR5eRobot` (context
  manager, ``move_l``, ``get_state``, ``home``) entirely in **dry-run** mode, so
  no real socket is ever opened and no real robot is commanded.
* ``force_control_example`` demonstrates
  :meth:`~ur5e_control.force.controller.ForceController.approach_until_force`
  against a scripted :class:`~ur5e_control.force.sensor.MockForceSensor` and a
  mock motion controller, again touching no hardware.

These tests assert the two acceptance criteria from the plan (Task 11): each
example's ``main`` runs to completion under dry-run/mock **without opening a real
socket and without raising**. We guard against accidental real I/O by patching
:func:`socket.socket` to fail loudly if anything tries to open a connection.

Units/frames (matching the locked interface): meters and radians, UR base frame.
"""

import importlib

import pytest


@pytest.fixture
def no_real_sockets(monkeypatch):
    """Make any attempt to open a real socket raise, to prove dry-run safety.

    The examples must run with no robot present; if a code path tried to open a
    TCP socket (upload a script, bind the state port, connect to the
    controller), this fixture turns that into a loud failure instead of a slow
    timeout or a confusing connection error.
    """

    import socket as socket_mod

    def _boom(*_args, **_kwargs):
        raise AssertionError(
            "example tried to open a real socket; it must run fully in dry-run/mock"
        )

    monkeypatch.setattr(socket_mod, "socket", _boom)
    return socket_mod


def test_move_example_imports():
    """move_example imports cleanly and exposes a callable main()."""
    mod = importlib.import_module("ur5e_control.examples.move_example")
    assert hasattr(mod, "main") and callable(mod.main)


def test_force_control_example_imports():
    """force_control_example imports cleanly and exposes a callable main()."""
    mod = importlib.import_module("ur5e_control.examples.force_control_example")
    assert hasattr(mod, "main") and callable(mod.main)


def test_move_example_runs_in_dry_run(no_real_sockets, capsys):
    """move_example.main(dry_run=True) completes without real sockets or errors."""
    mod = importlib.import_module("ur5e_control.examples.move_example")
    # Must not raise, and must not open a real socket (no_real_sockets fixture).
    mod.main(dry_run=True)
    out = capsys.readouterr().out
    # The preview should actually describe what it would do (non-empty output).
    assert out.strip(), "dry-run move example produced no output"


def test_force_control_example_runs_with_mock(no_real_sockets, capsys):
    """force_control_example.main() completes against the mock sensor/motion."""
    mod = importlib.import_module("ur5e_control.examples.force_control_example")
    mod.main()
    out = capsys.readouterr().out
    assert out.strip(), "force control example produced no output"


def test_python_control_imports():
    """python_control imports cleanly and exposes a callable main()."""
    mod = importlib.import_module("ur5e_control.examples.python_control")
    assert hasattr(mod, "main") and callable(mod.main)


def test_python_control_runs_in_dry_run(no_real_sockets, capsys):
    """python_control.main(dry_run=True) completes without real sockets or errors."""
    mod = importlib.import_module("ur5e_control.examples.python_control")
    mod.main(dry_run=True)
    assert capsys.readouterr().out.strip(), "dry-run python_control produced no output"


def test_python_with_gui_imports():
    """python_with_gui imports cleanly (it would bind a port if run, so don't)."""
    mod = importlib.import_module("ur5e_control.examples.python_with_gui")
    assert hasattr(mod, "main") and callable(mod.main)


def test_force_realrobot_imports():
    """force_realrobot imports cleanly and exposes a callable main()."""
    mod = importlib.import_module("ur5e_control.examples.force_realrobot")
    assert hasattr(mod, "main") and callable(mod.main)


def test_ft300_read_imports():
    """ft300_read imports cleanly and exposes a callable main() (needs hardware to run)."""
    mod = importlib.import_module("ur5e_control.examples.ft300_read")
    assert hasattr(mod, "main") and callable(mod.main)


def test_force_realrobot_runs_in_dry_run(no_real_sockets, capsys):
    """force_realrobot.main(dry_run=True) previews safely: no sockets, no errors."""
    mod = importlib.import_module("ur5e_control.examples.force_realrobot")
    mod.main(dry_run=True)
    assert capsys.readouterr().out.strip(), "dry-run force_realrobot produced no output"
