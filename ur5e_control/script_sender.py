"""Upload the URScript motion daemon to the UR5e controller.

The UR controller runs a URScript "daemon" that owns the realtime socket
transport (command protocol PC -> daemon, state stream daemon -> PC). Before any
motion command can be issued, that daemon source must be sent to the controller's
primary/secondary interface, which executes whatever URScript program it receives.

This module is intentionally transport-only: it moves raw bytes and does not
interpret poses, joints, or wrenches. All geometry (meters/radians, UR base
frame) lives inside the daemon script and in :mod:`ur5e_control.config`.

Typical use::

    from ur5e_control.config import RobotConfig
    from ur5e_control.script_sender import load_script, send_script

    script = load_script()              # packaged motion_daemon.script
    send_script(script, RobotConfig())  # upload to controller_ip:script_port
"""

from __future__ import annotations

import socket
from pathlib import Path

from ur5e_control.config import RobotConfig

__all__ = ["DEFAULT_SCRIPT_PATH", "load_script", "render_daemon", "send_script"]

# Path to the packaged URScript daemon, resolved relative to this file so it
# works regardless of the current working directory. The daemon file itself is
# created by a separate task; load_script simply reads whatever lives here.
DEFAULT_SCRIPT_PATH: Path = Path(__file__).resolve().parent / "urscript" / "motion_daemon.script"

# Seconds to wait when establishing the TCP connection to the controller.
_CONNECT_TIMEOUT_S = 5.0


def load_script(path: str | Path = DEFAULT_SCRIPT_PATH) -> bytes:
    """Read a URScript daemon file from disk and return its raw bytes.

    The file is read in binary mode and returned verbatim, so it can be sent to
    the controller unchanged (the UR primary interface executes the bytes as a
    URScript program). No encoding/normalisation is applied.

    Args:
        path: Filesystem path to the URScript source. Defaults to the packaged
            ``ur5e_control/urscript/motion_daemon.script``.

    Returns:
        The exact file contents as ``bytes``.

    Raises:
        OSError: If the file does not exist or cannot be read (this includes
            :class:`FileNotFoundError`).
    """
    return Path(path).read_bytes()


def render_daemon(
    config: RobotConfig = RobotConfig(),
    path: str | Path = DEFAULT_SCRIPT_PATH,
) -> bytes:
    """Render the daemon template, injecting the PC endpoint and home pose.

    The packaged ``motion_daemon.script`` is a template: the address the daemon
    dials **back** to (the PC) and the configured home pose are placeholders, so
    they always agree with the :class:`RobotConfig` the rest of the library uses
    (no second place to keep in sync). The substitutions are:

    * ``{{PC_HOST}}``    -> ``config.pc_host``    (where the daemon connects back)
    * ``{{STATE_PORT}}`` -> ``config.state_port``
    * ``{{HOME_POSE}}``  -> ``p[x, y, z, rx, ry, rz]`` from ``config.home_pose``
    * ``{{FORCE_MODE_ENABLED}}`` -> ``True``/``False`` from
      ``config.force_mode_enabled`` (arms the daemon's cmd 4/7 force branches;
      ``str(bool)`` renders the exact URScript boolean literal)
    * ``{{FORCE_MODE_DAMPING}}`` / ``{{FORCE_MODE_GAIN_SCALING}}`` -> the
      ``force_mode`` stability tuning from ``config`` (applied via
      ``force_mode_set_damping`` / ``force_mode_set_gain_scaling``).

    Note ``controller_ip``/``script_port`` are *not* in the daemon — those are
    where the PC uploads *to* (see :func:`send_script`), not where the daemon
    connects back.

    Args:
        config: Configuration supplying ``pc_host``, ``state_port``, ``home_pose``,
            ``force_mode_enabled``, ``force_mode_damping``,
            ``force_mode_gain_scaling``.
        path: Daemon template path. Defaults to the packaged daemon.

    Returns:
        Ready-to-upload URScript as ``bytes`` with all placeholders filled.
    """
    text = Path(path).read_text(encoding="utf-8")
    home = ", ".join(repr(float(v)) for v in config.home_pose)
    for token, value in (
        ("{{PC_HOST}}", str(config.pc_host)),
        ("{{STATE_PORT}}", str(int(config.state_port))),
        ("{{HOME_POSE}}", f"p[{home}]"),
        # URScript booleans are `True`/`False`; str(bool) matches exactly.
        ("{{FORCE_MODE_ENABLED}}", str(bool(config.force_mode_enabled))),
        ("{{FORCE_MODE_DAMPING}}", repr(float(config.force_mode_damping))),
        ("{{FORCE_MODE_GAIN_SCALING}}", repr(float(config.force_mode_gain_scaling))),
    ):
        text = text.replace(token, value)
    return text.encode("utf-8")


def send_script(
    script_bytes: bytes,
    config: RobotConfig = RobotConfig(),
    dry_run: bool = False,
) -> None:
    """Upload URScript bytes to the UR controller over TCP.

    Opens a TCP connection to ``config.controller_ip:config.script_port``, sends
    ``script_bytes`` in full, and closes the connection. The controller's primary
    interface executes the received program (the motion daemon).

    Args:
        script_bytes: The raw URScript program to upload (e.g. the result of
            :func:`load_script`).
        config: Robot configuration providing the controller endpoint. Defaults
            to a fresh :class:`RobotConfig`.
        dry_run: If ``True``, no socket is opened and nothing is sent; the call
            returns immediately. Useful for offline validation/testing.

    Returns:
        ``None``.

    Raises:
        OSError: If the connection fails or the send is interrupted (only when
            ``dry_run`` is ``False``).
    """
    if dry_run:
        return None

    endpoint = (config.controller_ip, config.script_port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.settimeout(_CONNECT_TIMEOUT_S)
        sock.connect(endpoint)
        sock.sendall(script_bytes)
    finally:
        sock.close()

    return None
