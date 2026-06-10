# `ur5e_control.connection`

Socket transport layer. `RobotConnection` owns the raw TCP transport between this
PC and the URScript motion daemon. **The PC is the server**: the daemon connects
*back* to `pc_host:state_port`. The single accepted connection is bidirectional
and is used both to **receive** the state stream (daemon -> PC) and **send**
command tuples (PC -> daemon).

This module deliberately knows **nothing** about motion semantics, the command
protocol, frame formats, units, or coordinate frames. It only moves bytes and
hands the latest received string back to the caller.

## Threading model

* A single background daemon thread (`_recv_loop`) owns the listening socket. It
  accepts the daemon, reads continuously into the latest-state buffer, and on
  disconnect loops back to `accept` (reconnect on broken pipe).
* `send` is callable from any thread and is serialized with a send lock.
* `latest_state` is callable from any thread and reads the most recent complete
  frame under a state lock.

A `dry_run` flag skips all real sockets: `start`, `send`, and `close` simply log;
`latest_state` always returns `""`.

---

## `class RobotConnection`

Bidirectional ASCII socket transport to the UR5e motion daemon.

### `__init__(self, config: RobotConfig = RobotConfig(), dry_run: bool = False) -> None`

* **Parameters.**
  * `config` — supplies `pc_host` and `state_port` for the bound endpoint
    (network only; no units/frames interpreted here).
  * `dry_run` — when `True`, no real sockets are opened.
* Performs no I/O; binding/listening happens at `start()`.

### `start(self) -> None`

**Purpose.** Bind, listen, and begin accepting the daemon in the background.

* Binds `pc_host:state_port` with `SO_REUSEADDR`, listens, then launches the
  background receive thread which accepts the daemon and reads its state stream.
  Returns immediately; the daemon may connect at any later point.
* In `dry_run` mode only logs (no thread or socket).
* **Exceptions.** Propagates `OSError` from `bind`/`listen` (e.g. address in
  use) when not in `dry_run`.

### `close(self) -> None`

**Purpose.** Shut down cleanly: stop the receive loop and release all sockets.

* Signals the thread to stop, closes the accepted daemon connection and the
  listening socket, and joins the thread. **Idempotent** — calling it more than
  once, or before `start`, is safe.
* In `dry_run` mode only logs.

### `send(self, msg: str) -> None`

**Purpose.** Send an ASCII command string to the daemon (thread-safe).

* **Parameters.** `msg` — the exact ASCII payload to transmit. Sent **verbatim**
  (no framing, number encoding, or unit/frame interpretation).
* The string is ASCII-encoded and written atomically with respect to other
  `send` callers (lock-guarded), so concurrent senders never interleave bytes.
* If the daemon is not currently connected, the send is **dropped with a
  warning**. On a broken pipe the connection is dropped so the receive loop
  re-accepts a fresh daemon.
* In `dry_run` mode only logs.

### `latest_state(self) -> str`

**Purpose.** Return the most recently received complete state frame (thread-safe).

* **Returns.** The raw frame string exactly as received (no parsing), or `""` if
  no complete frame has been received yet (and always in `dry_run` mode). Reads
  are lock-guarded so a concurrent update never yields a torn string.

### `is_connected(self) -> bool`

**Purpose.** Return `True` if a daemon is currently connected. Always `False` in
`dry_run` mode.

### Context manager

`__enter__` calls `start()` and returns `self`; `__exit__(*exc_info)` calls
`close()`.

### Usage example

```python
from ur5e_control.config import RobotConfig
from ur5e_control.connection import RobotConnection

cfg = RobotConfig()

# Real transport: PC listens, daemon connects back.
with RobotConnection(cfg) as conn:        # __enter__ -> start()
    conn.send("(3, 0,0,0,0,0,0, 0,0,0)")  # home tuple, sent verbatim
    raw = conn.latest_state()             # "" until a frame arrives
# __exit__ -> close()

# Offline preview, no sockets:
conn = RobotConnection(cfg, dry_run=True)
conn.start()
conn.send("(0, ...)")        # logged, not transmitted
assert conn.latest_state() == ""
conn.close()
```
