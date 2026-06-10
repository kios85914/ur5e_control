# `ur5e_control.motion`

Motion command encoding and execution. `MotionController` turns high-level motion
requests (move to a Cartesian pose, move to a joint configuration, stop, go home)
into the fixed-format command tuples understood by the URScript daemon, applies
the world<->UR frame transform and safety checks, pushes the encoded string
through a `RobotConnection`, and (optionally) blocks until convergence.

## Units and frames

* Positions in **meters**, rotations/joints in **radians**.
* A *pose* is `[x, y, z, rx, ry, rz]` (axis-angle); *joints* are `[j0..j5]`.
* Cartesian inputs to `move_l` are in the **world frame**; they are converted to
  the **UR base frame** via `RobotConfig.world_to_ur` before any safety check or
  encoding. Joint inputs to `move_j` are not frame-dependent and are used as-is.

## Command protocol emitted (PC -> daemon)

Fixed 10-float ASCII tuple, read by the daemon with
`socket_read_ascii_float(10, ...)`:

```
"(cmd, a0, a1, a2, a3, a4, a5, accel, vel, time)"
```

| cmd | name | a0..a5 | accel / vel / time |
|---|---|---|---|
| 0 | moveL | pose `x, y, z, rx, ry, rz` (m, rad) | m/s^2, m/s, s |
| 1 | moveJ | joints `j0..j5` (rad) | rad/s^2, rad/s, s |
| 2 | stop | `a0` = decel (m/s^2); `a1..a5` = 0 | 0, 0, 0 |
| 3 | home | all 0 (daemon uses configured home pose) | accel, vel, s |

Blocking moves poll `RobotConnection.latest_state`, parse with `parse_state`, and
exit once every TCP-pose component is within `RobotConfig.convergence_tol` of the
(UR-base-frame) target.

---

## `class MotionController`

Encode, validate, send, and (optionally) await UR5e motion commands. It depends
only on a `connection` exposing `send(str)` and `latest_state() -> str` (the real
`RobotConnection` or any compatible mock).

### `__init__(self, connection: RobotConnection, config: RobotConfig = RobotConfig()) -> None`

* **Parameters.**
  * `connection` — transport providing `send(msg: str) -> None` and
    `latest_state() -> str`. Strings produced here are sent verbatim.
  * `config` — supplies frame transforms, motion defaults, safety limits, and
    `convergence_tol`.

### `encode_command(cmd, payload, accel, vel, time) -> str`  *(static)*

Signature: `encode_command(cmd: int, payload: Sequence[float], accel: float, vel: float, time: float) -> str`

**Purpose.** Encode one command into the fixed 10-field ASCII tuple string.

* **Parameters.**
  * `cmd` — opcode (0 moveL, 1 moveJ, 2 stop, 3 home, 4 force).
  * `payload` — exactly six floats (`a0..a5`); meaning depends on `cmd`.
  * `accel` — acceleration (m/s^2 or rad/s^2).
  * `vel` — speed (m/s or rad/s).
  * `time` — move/blend duration (s).
* **Returns.** Exactly `"(cmd, a0, a1, a2, a3, a4, a5, accel, vel, time)"` with
  comma-space separators. `cmd` is rendered as an integer; the nine floats use
  Python's shortest round-trip representation (e.g. `0.1`, `0.0`, `-3.14`).
* **Exceptions.** `ValueError` if `payload` is not exactly six values.

### `move_l(pose, speed=None, accel=None, blocking=True, relative=False) -> None`

Signature: `move_l(self, pose: Sequence[float], speed: Optional[float] = None, accel: Optional[float] = None, blocking: bool = True, relative: bool = False) -> None`

**Purpose.** Move the TCP linearly to a Cartesian pose (cmd=0 moveL).

* **Parameters.**
  * `pose` — **world-frame** target (or delta if `relative`)
    `[x, y, z, rx, ry, rz]` (m, rad).
  * `speed` — Cartesian speed (m/s); `None` uses `config.default_speed`. Clamped
    to `config.max_speed`; must be > 0.
  * `accel` — Cartesian acceleration (m/s^2); `None` uses `config.default_accel`.
  * `blocking` — if `True`, poll state until within `config.convergence_tol` of
    the (UR-frame) target.
  * `relative` — if `True`, `pose` is a world-frame delta added to the current
    TCP pose (read from the latest daemon state, converted to world frame).
* **Behavior.** Validates/clamps speed first, then converts world -> UR
  (`world_to_ur`), workspace-checks the UR-frame pose, encodes, and sends.
* **Exceptions.**
  * `WorkspaceViolation` — malformed or out-of-workspace target (nothing sent).
  * `SpeedViolation` — `speed` not strictly positive (nothing sent).
  * `ValueError` — `relative=True` but no current state is available yet.
  * `TimeoutError` — blocking move does not converge within the poll ceiling.

### `move_j(joints, speed=None, accel=None, blocking=True) -> None`

Signature: `move_j(self, joints: Sequence[float], speed: Optional[float] = None, accel: Optional[float] = None, blocking: bool = True) -> None`

**Purpose.** Move to a joint configuration (cmd=1 moveJ).

* **Parameters.**
  * `joints` — target joint angles `[j0..j5]` (rad). Not frame-dependent.
  * `speed` — joint speed (rad/s); `None` uses `config.default_speed`. Clamped to
    `config.max_speed`; must be > 0.
  * `accel` — joint acceleration (rad/s^2); `None` uses `config.default_accel`.
  * `blocking` — if `True`, poll until the TCP pose has settled (the joint move
    effectively stopped) within `config.convergence_tol`.
* **Exceptions.**
  * `JointLimitViolation` — malformed or out-of-range joints (nothing sent).
  * `SpeedViolation` — `speed` not strictly positive (nothing sent).
  * `TimeoutError` — blocking move does not settle within the poll ceiling.

### `stop(deceleration: float = 2.0) -> None`

**Purpose.** Command an immediate controlled stop (cmd=2). Non-blocking.

* **Parameters.** `deceleration` — stop deceleration (m/s^2; > 0 to be useful),
  placed in `a0`; `a1..a5` and accel/vel/time are zero.

### `home(speed=None, accel=None, blocking=True) -> None`

Signature: `home(self, speed: Optional[float] = None, accel: Optional[float] = None, blocking: bool = True) -> None`

**Purpose.** Move to the configured home pose (cmd=3). Sends an all-zero payload;
the daemon substitutes its configured home pose (`config.home_pose`, UR base
frame).

* **Parameters.**
  * `speed` — m/s; `None` uses `config.default_speed`. Clamped; must be > 0.
  * `accel` — m/s^2; `None` uses `config.default_accel`.
  * `blocking` — if `True`, poll until within `config.convergence_tol` of
    `config.home_pose`.
* **Exceptions.** `SpeedViolation` if `speed` not strictly positive (nothing
  sent); `TimeoutError` if a blocking home does not converge.

### Usage example

```python
from ur5e_control.config import RobotConfig
from ur5e_control.connection import RobotConnection
from ur5e_control.motion import MotionController

cfg = RobotConfig()
conn = RobotConnection(cfg)
conn.start()
mc = MotionController(conn, cfg)

# Encode without sending (also usable directly):
msg = MotionController.encode_command(0, [-0.1, -0.3, 0.2, 0.0, -3.14, 0.0], 0.1, 0.1, 2.0)
# -> "(0, -0.1, -0.3, 0.2, 0.0, -3.14, 0.0, 0.1, 0.1, 2.0)"

mc.move_l([0.1, 0.3, 0.2, 0.0, -3.14, 0.0])          # world frame, m/rad
mc.move_l([0.0, 0.0, -0.05, 0, 0, 0], relative=True)  # 5 cm down, world delta
mc.move_j([0.0, -1.57, 1.57, 0.0, 1.57, 0.0])         # joints, rad
mc.stop()
mc.home()
conn.close()
```
