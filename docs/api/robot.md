# `ur5e_control.robot`

High-level UR5e robot facade — the public, user-facing entry point. `UR5eRobot`
composes the three lower layers (daemon uploader `script_sender`, transport
`RobotConnection`, motion encoder/executor `MotionController`) into a single
object with a small, stable API. It owns no geometry or protocol logic of its
own: every motion request is delegated to the `MotionController`, and state is
parsed from the raw frame the connection last received.

## Units and frames

* Positions in **meters**, rotations/joints in **radians**.
* A *pose* is `[x, y, z, rx, ry, rz]` (axis-angle); *joints* are `[j0..j5]`.
* Cartesian inputs to `move_l` are in the **world frame**; the underlying
  `MotionController` converts them to the **UR base frame** via
  `RobotConfig.world_to_ur`. Joint inputs to `move_j` are not frame-dependent.
  The `RobotState` returned by `get_state` is in the **UR base frame**.

## Lifecycle

* `connect()` uploads the motion daemon to the controller (via `send_script`)
  and then starts the connection so the daemon's state stream begins flowing
  back. `disconnect()` closes the connection.
* The object is a context manager: `with UR5eRobot(cfg) as robot:` connects on
  entry and disconnects on exit (even if the body raises).

---

## `class UR5eRobot`

### `__init__(self, config: RobotConfig = RobotConfig()) -> None`

* **Parameters.** `config` — supplies network endpoints, motion defaults, safety
  limits, and the world<->UR frame transform. Defaults to a fresh `RobotConfig`.
* Wires up the connection and motion controller but performs **no I/O**.
* **Attribute.** `config` is stored as a public attribute (`robot.config`).

### `connect(self) -> None`

Reads the packaged URScript daemon with `load_script`, uploads it to the
controller with `send_script` (to `config.controller_ip:config.script_port`), and
then starts the `RobotConnection` so the daemon's state stream is received. The
upload must precede the connection start so the daemon is running before it
connects back to the PC.

* **Exceptions.** `OSError` propagated from `load_script` / `send_script` /
  socket bind.

### `disconnect(self) -> None`

Closes the state connection and releases its sockets. **Idempotent** — safe to
call more than once or before `connect`.

### `move_l(pose, speed=None, accel=None, blocking=True, relative=False, move_time=None) -> None`

Signature: `move_l(self, pose: Sequence[float], speed: Optional[float] = None, accel: Optional[float] = None, blocking: bool = True, relative: bool = False, move_time: Optional[float] = None) -> None`

Move the TCP linearly to a Cartesian pose (delegates to `MotionController`).

* **Parameters.**
  * `pose` — world-frame target (or delta if `relative`) `[x, y, z, rx, ry, rz]`
    (m, rad).
  * `speed` — Cartesian speed (m/s); `None` uses `config.default_speed`.
  * `accel` — Cartesian acceleration (m/s^2); `None` uses `config.default_accel`.
  * `blocking` — if `True`, block until the move converges on the target.
  * `relative` — if `True`, `pose` is a delta on the current world pose.
  * `move_time` — move duration in seconds (URScript `t`); `None` uses
    `config.default_move_time`. **If > 0 it overrides `speed`/`accel`** — the move
    takes exactly `move_time` seconds; `0.0` lets speed govern. Must be >= 0.
* **Exceptions.** `WorkspaceViolation`, `SpeedViolation`, `ValueError`
  (relative with no state, or negative `move_time`), `TimeoutError`.

### `move_j(joints, speed=None, accel=None, blocking=True, move_time=None) -> None`

Signature: `move_j(self, joints: Sequence[float], speed: Optional[float] = None, accel: Optional[float] = None, blocking: bool = True, move_time: Optional[float] = None) -> None`

Move to a joint configuration (delegates to `MotionController`).

* **Parameters.**
  * `joints` — target joint angles `[j0..j5]` (rad, frame-independent).
  * `speed` — joint speed (rad/s); `None` uses `config.default_speed`.
  * `accel` — joint acceleration (rad/s^2); `None` uses `config.default_accel`.
  * `blocking` — if `True`, block until the joint move settles.
  * `move_time` — move duration in seconds (URScript `t`); `None` uses
    `config.default_move_time`. **If > 0 it overrides `speed`/`accel`**; `0.0`
    lets speed govern. Must be >= 0.
* **Exceptions.** `JointLimitViolation`, `SpeedViolation`, `ValueError`
  (negative `move_time`), `TimeoutError`.

### `stop(self) -> None`

Command an immediate controlled stop (cmd=2, via `MotionController`, using the
controller's default deceleration).

### `home(self, speed=None, accel=None, blocking=True, move_time=None) -> None`

Move to the configured home pose (cmd=3, via `MotionController`). The daemon homes
to `config.home_pose` (UR base frame, m/rad). `move_time` works as in `move_l`
(URScript `t`; > 0 overrides `speed`/`accel`).

### `get_state(self) -> RobotState`

Return the latest robot state as a parsed `RobotState`. Reads the most recent raw
state frame (`RobotConnection.latest_state`) and parses it (`parse_state`). All
quantities are in the **UR base frame** (m/rad; velocities m/s, rad/s; forces N,
Nm).

* **Returns.** The parsed `RobotState` snapshot.
* **Exceptions.** `ValueError` if no complete state frame is available yet or the
  latest frame is malformed (propagated from `parse_state`).

### Context manager

`__enter__` connects and returns `self`; `__exit__(*exc_info)` disconnects (runs
even if the body raised).

### Usage example

```python
from ur5e_control.config import RobotConfig
from ur5e_control.robot import UR5eRobot

with UR5eRobot(RobotConfig()) as robot:          # connect on entry
    robot.move_l([0.1, 0.3, 0.2, 0.0, -3.14, 0.0])  # world frame, m/rad
    robot.move_j([0.0, -1.57, 1.57, 0.0, 1.57, 0.0]) # joints, rad
    state = robot.get_state()                    # UR base frame
    print(state.tcp_pose, state.wrench)
    robot.home()
# disconnect on exit

# Manual lifecycle:
robot = UR5eRobot()
robot.connect()
try:
    robot.move_l([0.0, 0.0, -0.05, 0, 0, 0], relative=True)  # 5 cm down
finally:
    robot.disconnect()
```
