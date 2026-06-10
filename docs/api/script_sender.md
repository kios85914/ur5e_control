# `ur5e_control.script_sender`

Upload the URScript motion daemon to the UR5e controller. The UR controller runs
a URScript "daemon" that owns the realtime socket transport (command protocol
PC -> daemon, state stream daemon -> PC). Before any motion command can be issued,
that daemon source must be sent to the controller's primary/secondary interface,
which executes whatever URScript program it receives.

This module is intentionally transport-only: it moves raw bytes and does not
interpret poses, joints, or wrenches. All geometry (meters/radians, UR base
frame) lives inside the daemon script and in `ur5e_control.config`.

## Module attribute

* `DEFAULT_SCRIPT_PATH: Path` — path to the packaged URScript daemon, resolved
  relative to this module:
  `ur5e_control/urscript/motion_daemon.script`. Resolved at import time so it
  works regardless of the current working directory.

---

## `load_script(path: str | Path = DEFAULT_SCRIPT_PATH) -> bytes`

**Purpose.** Read a URScript daemon file from disk and return its raw bytes.

* **Parameters.** `path` — filesystem path to the URScript source. Defaults to
  the packaged `motion_daemon.script`.
* **Returns.** The exact file contents as `bytes` (read in binary mode, returned
  verbatim — no encoding/normalisation), so it can be sent to the controller
  unchanged.
* **Exceptions.** `OSError` if the file does not exist or cannot be read
  (includes `FileNotFoundError`).

## `send_script(script_bytes: bytes, config: RobotConfig = RobotConfig(), dry_run: bool = False) -> None`

**Purpose.** Upload URScript bytes to the UR controller over TCP.

* **Parameters.**
  * `script_bytes` — the raw URScript program to upload (e.g. the result of
    `load_script()`).
  * `config` — supplies the controller endpoint
    (`controller_ip:script_port`). Defaults to a fresh `RobotConfig`.
  * `dry_run` — if `True`, no socket is opened and nothing is sent; the call
    returns immediately (useful for offline validation/testing).
* **Behavior.** Opens a TCP connection to `config.controller_ip:config.script_port`
  (5-second connect timeout), sends `script_bytes` in full, and closes the
  connection. The controller's primary interface executes the received program.
* **Returns.** `None`.
* **Exceptions.** `OSError` if the connection fails or the send is interrupted
  (only when `dry_run` is `False`).

### Usage example

```python
from ur5e_control.config import RobotConfig
from ur5e_control.script_sender import load_script, send_script, DEFAULT_SCRIPT_PATH

script = load_script()                 # read packaged motion_daemon.script
send_script(script, RobotConfig())     # upload to controller_ip:script_port

# Offline validation (no socket opened):
send_script(script, RobotConfig(), dry_run=True)

# Custom script path:
script = load_script("/path/to/custom_daemon.script")
```
