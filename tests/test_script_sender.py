"""Tests for ur5e_control.script_sender — load and upload the URScript daemon.

The script sender reads the URScript daemon source file from disk and uploads it
to the UR controller over TCP. The controller endpoint is taken from
:class:`RobotConfig` (``controller_ip:script_port``); positions/units are not
relevant here — this module only moves raw bytes.
"""

from unittest import mock

import pytest

from ur5e_control.config import RobotConfig
from ur5e_control.script_sender import DEFAULT_SCRIPT_PATH, load_script, send_script


# ---------------------------------------------------------------------------
# load_script
# ---------------------------------------------------------------------------
def test_load_script_returns_bytes(tmp_path):
    """load_script reads a file and returns its exact contents as bytes."""
    payload = b"def daemon():\n  textmsg(\"hi\")\nend\n"
    script_file = tmp_path / "motion_daemon.script"
    script_file.write_bytes(payload)

    result = load_script(str(script_file))

    assert isinstance(result, bytes)
    assert result == payload


def test_load_script_default_path_is_packaged_daemon():
    """The default path points at the packaged motion_daemon.script."""
    assert str(DEFAULT_SCRIPT_PATH).endswith("urscript/motion_daemon.script")


def test_load_script_missing_file_raises():
    """A non-existent path raises (FileNotFoundError is an OSError)."""
    with pytest.raises(OSError):
        load_script("/no/such/file/motion_daemon.script")


# ---------------------------------------------------------------------------
# send_script — real (mocked) socket
# ---------------------------------------------------------------------------
def test_send_script_connects_to_controller_and_sends_bytes():
    """send_script connects to controller_ip:script_port and sends the bytes."""
    cfg = RobotConfig(controller_ip="10.1.2.3", script_port=30001)
    script_bytes = b"def daemon():\nend\n"

    with mock.patch("ur5e_control.script_sender.socket.socket") as mock_socket_cls:
        mock_sock = mock_socket_cls.return_value

        send_script(script_bytes, cfg)

    # A socket was created and used to connect to the configured endpoint.
    mock_socket_cls.assert_called_once()
    mock_sock.connect.assert_called_once_with(("10.1.2.3", 30001))

    # The exact bytes were transmitted (sendall preferred, but accept send too).
    sent = b""
    if mock_sock.sendall.called:
        for call in mock_sock.sendall.call_args_list:
            sent += call.args[0]
    if mock_sock.send.called:
        for call in mock_sock.send.call_args_list:
            sent += call.args[0]
    assert sent == script_bytes


def test_send_script_closes_socket():
    """The socket is closed after sending (directly or via context manager)."""
    cfg = RobotConfig()
    with mock.patch("ur5e_control.script_sender.socket.socket") as mock_socket_cls:
        mock_sock = mock_socket_cls.return_value

        send_script(b"x", cfg)

    closed = mock_sock.close.called or mock_sock.__exit__.called
    assert closed, "expected the socket to be closed after sending"


def test_send_script_uses_default_config():
    """Calling without an explicit config uses RobotConfig() defaults."""
    default = RobotConfig()
    with mock.patch("ur5e_control.script_sender.socket.socket") as mock_socket_cls:
        mock_sock = mock_socket_cls.return_value

        send_script(b"data")

    mock_sock.connect.assert_called_once_with(
        (default.controller_ip, default.script_port)
    )


# ---------------------------------------------------------------------------
# send_script — dry run
# ---------------------------------------------------------------------------
def test_send_script_dry_run_opens_no_socket():
    """dry_run=True must not open a socket at all."""
    cfg = RobotConfig()
    with mock.patch("ur5e_control.script_sender.socket.socket") as mock_socket_cls:
        send_script(b"anything", cfg, dry_run=True)

    mock_socket_cls.assert_not_called()


def test_send_script_dry_run_returns_none():
    """dry_run returns None and does not raise even with empty payload."""
    assert send_script(b"", RobotConfig(), dry_run=True) is None
