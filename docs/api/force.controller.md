# `ur5e_control.force.controller`

Force-guided approach control for the UR5e (move-until-force, then hold).

> **Pending hardware validation / tuning.** This controller is a *pure control
> law* exercised today only with a mock motion controller and a scripted
> `MockForceSensor`. No part of it has been run against a physically mounted
> Robotiq FT 300. The step size, the force-magnitude threshold behaviour near
> contact, the `stiffness` value, and the `max_travel` safety budget **must be
> re-tuned on the real robot** before any contact task is trusted. Until then
> treat all numeric defaults as placeholders.

`ForceController` implements `approach_until_force`: drive the TCP in small
relative linear steps along a commanded direction while watching the
force/torque sensor; the moment the measured force **projected onto that
direction** reaches the target, stop approaching and transition to a held
URScript *force-mode* command (`cmd=4`).

## Units & frames

* The approach `direction` is a 3-vector in the **UR base frame** (the same frame
  the sensor wrench and TCP pose live in); it is normalised internally so its
  magnitude is irrelevant.
* `target_n` is a force in **newtons (N)**; `max_travel` and the internal step
  size are in **meters (m)**; `stiffness` is the force-mode compliance parameter
  handed straight to the daemon.
* Sensor wrenches are `[fx, fy, fz, tx, ty, tz]` (N, Nm), UR base frame; only the
  linear force triplet `[fx, fy, fz]` is used.

## Force-mode command (PC -> daemon), `cmd=4`

```
"(4, dx, dy, dz, target_n, stiffness, max_travel, accel, vel, time)"
```

`(dx, dy, dz)` is the *unit* approach direction, `target_n` the target force (N),
`stiffness` the compliance, `max_travel` the travel ceiling (m). This matches
`encode_command(4, [dx, dy, dz, target_n, stiffness, max_travel], accel, vel, time)`.

---

## `class ForceController`

Move-until-force-then-hold controller built on a motion controller + sensor. It
depends only on a `motion` object exposing `move_l`, the static `encode_command`,
and a command-send path (a public `send_command` if present, else the underlying
connection's `send`), and on a `ForceSensor` exposing `read()`. Fully mockable.

### `__init__(self, motion, sensor: ForceSensor, config: RobotConfig = RobotConfig()) -> None`

* **Parameters.**
  * `motion` — motion controller providing `move_l(pose, ..., relative=...)`, the
    static `encode_command(cmd, payload, accel, vel, time)`, and a command-send
    path. Approach steps are issued as relative linear moves in the UR base frame
    (m/rad).
  * `sensor` — force/torque sensor whose `read()` returns a 6-element wrench
    `[fx, fy, fz, tx, ty, tz]` (N, Nm), UR base frame.
  * `config` — supplies motion defaults (speed/accel/time). Defaults to a fresh
    `RobotConfig`.

### `approach_until_force(direction, target_n, stiffness, max_travel, step=0.005) -> None`

Signature: `approach_until_force(self, direction: Sequence[float], target_n: float, stiffness: float, max_travel: float, step: float = 0.005) -> None`

**Purpose.** Approach along `direction` until contact, then hold with force mode.

Drives the TCP in small relative linear steps along the (normalised) `direction`
while polling `sensor.read()` on every iteration. The decision variable is the
measured force **projected onto the unit direction** (`f . d_hat`, in N) —
perpendicular forces do not trigger contact. The instant that projection reaches
`target_n`, the approach stops **immediately** (no further step is issued, so the
contact step is never overshot) and a held force-mode command (`cmd=4`) is sent.
The contact check happens **before** each step.

* **Parameters.**
  * `direction` — approach direction `[dx, dy, dz]` in the UR base frame.
    Magnitude irrelevant (normalised); only the orientation matters.
  * `target_n` — target contact force in **newtons** measured along `direction`.
    Must be strictly positive.
  * `stiffness` — force-mode compliance/stiffness parameter passed straight
    through to the daemon's force mode.
  * `max_travel` — maximum distance to travel along `direction` while searching
    for contact, in **meters**. Must be strictly positive; if contact is not
    reached within this budget the approach aborts (no force-mode command sent).
  * `step` — per-iteration approach increment in meters (PLACEHOLDER default,
    pending on-robot tuning). Must be strictly positive.
* **Held command.** Encodes (via the motion controller's `encode_command`)
  `a0..a2 = unit direction`, `a3 = target_n`, `a4 = stiffness`,
  `a5 = max_travel`, with the configured default accel/vel/move-time.
* **Exceptions.**
  * `ValueError` — `direction` is not a non-zero 3-vector, or `target_n`,
    `max_travel`, or `step` is not strictly positive.
  * `RuntimeError` — `max_travel` is consumed (or the internal iteration ceiling
    is hit) before the target force is reached. No force-mode hold is emitted in
    that case.

### The three force behaviors (recommended API)

Beyond the legacy `approach_until_force`, the controller exposes the three
behaviors used by the examples:

* `guarded_move(direction, speed, force_threshold_n, max_travel, ...)` — PC-side
  velocity move (`cmd=5` speedl) that **stops and holds** the instant the force
  along `direction` reaches the threshold. Not gated.
* `maintain_force(direction, target_n, speed_limit=0.05, max_travel=0.05, ..., frame="base")`
  — hand off to the controller's `force_mode` (`cmd=4`) to **keep a constant
  contact force**. The selection vector follows `direction` (e.g. `[0,0,-1]` →
  only Z compliant). `frame="base"` reads `direction` in the UR base frame;
  `frame="tool"` reads it in the TCP/tool frame (push along the tool's own axis
  whatever way it points). Gated by `FORCE_MODE_ENABLED`.
* `hold_compliant(compliant_axes=(1,1,1), stiffness=300.0, ..., frame="base")` —
  impedance spring about the **current** pose (`cmd=7`): hold here, yield to
  pushes. `frame="tool"` makes `compliant_axes` the tool axes frozen at entry.
  Gated.

> **Frame note.** `frame` rides in the command tuple's unused `vel` field
> (0.0 = base, 1.0 = tool); the daemon picks `force_mode`'s task frame
> accordingly. Default `"base"` matches `speedl` / `guarded_move` / `move_l`.

### `impedance_move(target, stiffness=300.0, speed_limit=0.05, max_deviation=0.02, accel=None) -> None`

**Purpose.** Impedance control **toward a target** (`cmd=8`). Unlike
`hold_compliant` (equilibrium = current pose), the spring's equilibrium is
`target`: the arm is pulled toward `target` with force `K*(target - x)` *and*
yields to external force on the way (block it → it gives; release → it resumes).
All three translation axes comply; orientation is held at entry. Non-blocking;
runs until `end_force()`. **Gated by `FORCE_MODE_ENABLED` — pending hardware
validation.**

* **Parameters.**
  * `target` — equilibrium position `[x, y, z]` (m, UR base frame).
  * `stiffness` — spring stiffness K (N/m, > 0).
  * `speed_limit` — compliant-axis speed cap (m/s).
  * `max_deviation` — per-axis error clamp (m); the spring force **saturates at
    `K * max_deviation`** so a far target cannot yank the arm (default K=300,
    max_deviation=0.02 → ~6 N ceiling).
  * `accel` — ramp accel (m/s^2); `None` uses the config default.
* **Exceptions.** `ValueError` if `target` is not a 3-vector or `stiffness <= 0`.

> **Note.** Impedance applies only while this command is active. `move_l` /
> `move_j` remain rigid position control — they do not yield. Exit impedance with
> `end_force()` to return to stiff control.

### Usage example

```python
from ur5e_control.config import RobotConfig
from ur5e_control.connection import RobotConnection
from ur5e_control.motion import MotionController
from ur5e_control.force.sensor import MockForceSensor
from ur5e_control.force.controller import ForceController

cfg = RobotConfig()
conn = RobotConnection(cfg)
mc = MotionController(conn, cfg)

# Mock sensor: no contact, then 5 N along +z on the 3rd read.
sensor = MockForceSensor([
    [0, 0, 0.0, 0, 0, 0],
    [0, 0, 2.0, 0, 0, 0],
    [0, 0, 5.0, 0, 0, 0],
])

fc = ForceController(mc, sensor, cfg)
# Approach straight down (-z, UR base frame) until 5 N, then hold.
fc.approach_until_force(
    direction=[0.0, 0.0, -1.0],
    target_n=5.0,
    stiffness=0.5,
    max_travel=0.05,   # 5 cm search budget
    step=0.005,        # 5 mm per step
)
```
