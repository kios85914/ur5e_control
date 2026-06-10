"""Tests for ur5e_control.robot — the UR5eRobot facade.

:class:`~ur5e_control.robot.UR5eRobot` is a thin composition layer over three
collaborators: the URScript uploader (:mod:`ur5e_control.script_sender`), the
socket transport (:class:`~ur5e_control.connection.RobotConnection`), and the
motion encoder/executor (:class:`~ur5e_control.motion.MotionController`). These
tests patch all three collaborators **in the robot module's namespace** so no
real sockets are opened and delegation can be asserted directly: the facade owns
no geometry of its own, it just wires calls through.

Units/frames (matching the locked interface): meters and radians, UR base frame
on the wire; world-frame Cartesian inputs are converted inside MotionController.
"""

from unittest import mock

import pytest

from ur5e_control.config import RobotConfig
from ur5e_control.robot import UR5eRobot
from ur5e_control.state import RobotState


# A representative raw daemon state frame (meters/radians, UR base frame). Only
# its parseability matters here; get_state() must turn it into a RobotState.
_RAW_STATE = (
    "p[0.1,0.2,0.3,0.0,-3.14,0.0]_p[0,0,0,0,0,0]_"
    "[0,-1.57,1.57,0,1.57,0]_p[1.0,2.0,3.0,0.1,0.2,0.3]+"
)


@pytest.fixture
def patched(monkeypatch):
    """Patch script_sender, RobotConnection, and MotionController in robot.py.

    Returns a namespace of mocks so each test can assert on the exact calls the
    facade makes. ``load_script``/``send_script`` are functions; ``RobotConnection``
    and ``MotionController`` are classes whose instances are the ``.return_value``
    of the patched class mock.
    """
    import ur5e_control.robot as robot_mod

    load_script = mock.Mock(name="load_script", return_value=b"SCRIPT-BYTES")
    send_script = mock.Mock(name="send_script")
    connection_cls = mock.Mock(name="RobotConnection")
    motion_cls = mock.Mock(name="MotionController")

    monkeypatch.setattr(robot_mod, "load_script", load_script)
    monkeypatch.setattr(robot_mod, "send_script", send_script)
    monkeypatch.setattr(robot_mod, "RobotConnection", connection_cls)
    monkeypatch.setattr(robot_mod, "MotionController", motion_cls)

    return mock.Mock(
        robot_mod=robot_mod,
        load_script=load_script,
        send_script=send_script,
        connection_cls=connection_cls,
        connection=connection_cls.return_value,
        motion_cls=motion_cls,
        motion=motion_cls.return_value,
    )


# ---------------------------------------------------------------------------
# Construction / defaults
# ---------------------------------------------------------------------------
def test_default_config_used_when_none_given(patched):
    """Constructing with no config yields a usable RobotConfig instance."""
    robot = UR5eRobot()
    assert isinstance(robot.config, RobotConfig)


def test_provided_config_is_kept(patched):
    """A supplied config is stored verbatim (no copy/replacement)."""
    cfg = RobotConfig(controller_ip="10.0.0.9")
    robot = UR5eRobot(cfg)
    assert robot.config is cfg


# ---------------------------------------------------------------------------
# connect / disconnect
# ---------------------------------------------------------------------------
def test_connect_uploads_daemon_then_starts_connection(patched):
    """connect() loads + sends the daemon script, then starts the connection."""
    robot = UR5eRobot()
    robot.connect()

    # The packaged daemon script is loaded and uploaded to the controller.
    patched.load_script.assert_called_once_with()
    patched.send_script.assert_called_once()
    sent_args, sent_kwargs = patched.send_script.call_args
    assert sent_args[0] == b"SCRIPT-BYTES"

    # The connection's receive loop is started.
    patched.connection.start.assert_called_once_with()


def test_connect_sends_script_before_starting_connection(patched):
    """The daemon upload must happen before the state listener starts."""
    order = []
    patched.send_script.side_effect = lambda *a, **k: order.append("send_script")
    patched.connection.start.side_effect = lambda *a, **k: order.append("start")

    UR5eRobot().connect()

    assert order == ["send_script", "start"]


def test_disconnect_closes_connection(patched):
    """disconnect() closes the underlying connection."""
    robot = UR5eRobot()
    robot.connect()
    robot.disconnect()
    patched.connection.close.assert_called_once_with()


# ---------------------------------------------------------------------------
# Context manager
# ---------------------------------------------------------------------------
def test_context_manager_connects_on_enter_and_returns_self(patched):
    """__enter__ connects and returns the robot itself."""
    robot = UR5eRobot()
    with robot as entered:
        assert entered is robot
        patched.send_script.assert_called_once()
        patched.connection.start.assert_called_once_with()


def test_context_manager_disconnects_on_exit(patched):
    """__exit__ disconnects (closes the connection) even on clean exit."""
    with UR5eRobot():
        pass
    patched.connection.close.assert_called_once_with()


def test_context_manager_disconnects_on_exception(patched):
    """__exit__ disconnects even when the body raises (no swallowing)."""
    with pytest.raises(RuntimeError):
        with UR5eRobot():
            raise RuntimeError("boom")
    patched.connection.close.assert_called_once_with()


# ---------------------------------------------------------------------------
# Motion delegation
# ---------------------------------------------------------------------------
def test_move_l_delegates_to_motion_controller(patched):
    """move_l forwards pose + kwargs to MotionController.move_l unchanged."""
    robot = UR5eRobot()
    pose = [0.1, 0.2, 0.3, 0.0, -3.14, 0.0]
    robot.move_l(pose, speed=0.05, accel=0.2, blocking=False, relative=True)

    patched.motion.move_l.assert_called_once_with(
        pose, speed=0.05, accel=0.2, blocking=False, relative=True
    )


def test_move_l_defaults_match_locked_signature(patched):
    """move_l(pose) uses blocking=True, relative=False, speed/accel None."""
    robot = UR5eRobot()
    pose = [0.1, 0.2, 0.3, 0.0, -3.14, 0.0]
    robot.move_l(pose)

    patched.motion.move_l.assert_called_once_with(
        pose, speed=None, accel=None, blocking=True, relative=False
    )


def test_move_j_delegates_to_motion_controller(patched):
    """move_j forwards joints + kwargs to MotionController.move_j unchanged."""
    robot = UR5eRobot()
    joints = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
    robot.move_j(joints, speed=0.1, accel=0.3, blocking=False)

    patched.motion.move_j.assert_called_once_with(
        joints, speed=0.1, accel=0.3, blocking=False
    )


def test_move_j_defaults_match_locked_signature(patched):
    """move_j(joints) uses blocking=True, speed/accel None."""
    robot = UR5eRobot()
    joints = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
    robot.move_j(joints)

    patched.motion.move_j.assert_called_once_with(
        joints, speed=None, accel=None, blocking=True
    )


# ---------------------------------------------------------------------------
# home / stop delegation
# ---------------------------------------------------------------------------
def test_home_delegates_to_motion_controller(patched):
    """home() delegates to MotionController.home (which sends cmd=3)."""
    robot = UR5eRobot()
    robot.home()
    patched.motion.home.assert_called_once_with()


def test_stop_delegates_to_motion_controller(patched):
    """stop() delegates to MotionController.stop (which sends cmd=2)."""
    robot = UR5eRobot()
    robot.stop()
    patched.motion.stop.assert_called_once_with()


# ---------------------------------------------------------------------------
# get_state
# ---------------------------------------------------------------------------
def test_get_state_parses_latest_connection_frame(patched):
    """get_state parses connection.latest_state() into a RobotState."""
    patched.connection.latest_state.return_value = _RAW_STATE
    robot = UR5eRobot()

    state = robot.get_state()

    patched.connection.latest_state.assert_called_once_with()
    assert isinstance(state, RobotState)
    assert state.tcp_pose == [0.1, 0.2, 0.3, 0.0, -3.14, 0.0]
    assert state.wrench == [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]


def test_get_state_raises_on_malformed_frame(patched):
    """A malformed/empty frame from the connection raises ValueError."""
    patched.connection.latest_state.return_value = ""
    robot = UR5eRobot()
    with pytest.raises(ValueError):
        robot.get_state()


# ---------------------------------------------------------------------------
# Wiring: collaborators are constructed with the same config
# ---------------------------------------------------------------------------
def test_collaborators_built_with_config(patched):
    """RobotConnection and MotionController are constructed from the config."""
    cfg = RobotConfig()
    robot = UR5eRobot(cfg)

    # RobotConnection is built with the config.
    assert patched.connection_cls.call_count == 1
    conn_args, conn_kwargs = patched.connection_cls.call_args
    assert cfg in conn_args or cfg in conn_kwargs.values()

    # MotionController is built with the connection and the config.
    assert patched.motion_cls.call_count == 1
    motion_args, motion_kwargs = patched.motion_cls.call_args
    all_motion = list(motion_args) + list(motion_kwargs.values())
    assert patched.connection in all_motion
    assert cfg in all_motion
