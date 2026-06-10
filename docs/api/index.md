# `ur5e_control` API Reference

Pure Python (3.10, standard library only) control library for the Universal
Robots UR5e over the URScript socket transport. This index links the per-module
references and shows how the layers fit together, the PC<->daemon command
protocol, and the daemon's state-stream format.

## Conventions used everywhere

* **Units:** distances in **meters**, angles in **radians**, velocities in m/s
  and rad/s, forces in **newtons (N)**, torques in **newton-metres (Nm)**.
* **Frames:** all daemon I/O and stored geometry are in the **UR base frame**.
  Cartesian *inputs* to `move_l` (and the facade) are in the **world frame** and
  are converted via `RobotConfig.world_to_ur` (which negates only x and y).
  `RobotConfig` is the single source of truth for the conversion — no hidden
  negation lives anywhere else.
* **Pose / joints:** a *pose* is `[x, y, z, rx, ry, rz]` (axis-angle
  orientation); *joints* are `[j0, j1, j2, j3, j4, j5]`.

## Module reference

| Layer | Module | What it does |
|---|---|---|
| Foundation | [`config`](config.md) | `RobotConfig`: endpoints, motion defaults, safety limits, world<->UR transform. |
| Foundation | [`state`](state.md) | `RobotState` dataclass + `parse_state(raw)` for daemon frames. |
| Foundation | [`safety`](safety.md) | Pure validators: workspace, joint limits, speed clamp + typed violations. |
| Transport | [`connection`](connection.md) | `RobotConnection`: PC-as-server bidirectional ASCII socket + background receive. |
| Transport | [`script_sender`](script_sender.md) | `load_script` / `send_script`: upload the URScript daemon to the controller. |
| Motion | [`motion`](motion.md) | `MotionController`: encode command tuples, apply frame/safety, send, await convergence. |
| Facade | [`robot`](robot.md) | `UR5eRobot`: user-facing entry point composing all layers. |
| Force | [`force.sensor`](force.sensor.md) | `ForceSensor` ABC + `MockForceSensor` + `RobotiqFT300` (pending hardware). |
| Force | [`force.controller`](force.controller.md) | `ForceController.approach_until_force`: move-until-force, then hold (pending hardware). |

## Layered architecture

Each layer depends only on those below it. `config` and `state` are the shared
foundation; `RobotConfig` is the single source of truth for units/frames/limits.

```
                    +-----------------------------+
                    |          UR5eRobot          |   facade  (robot.py)
                    |  connect / move_l / move_j  |
                    |  stop / home / get_state    |
                    +--------------+--------------+
                                   |
                    +--------------v--------------+
                    |       MotionController       |   motion  (motion.py)
                    |  encode_command, frame xform,|
                    |  safety checks, await conv.  |
                    +-----+-----------------+------+
                          |                 |
            +-------------v----+     +------v-------------+
            | RobotConnection  |     |   script_sender    |  transport
            | (socket, recv    |     | load/send daemon   |
            |  thread, send)   |     |  to controller     |
            +-------------+----+     +--------------------+
                          |
        +-----------------+------------------------------+
        |                 |                  |            |
   +----v----+      +-----v----+       +-----v----+  +----v-----+
   | config  |      |  state   |       |  safety  |  |  force.* |
   |Robot-   |      |RobotState|       |validators|  | sensor / |
   |Config   |      |parse_st. |       |          |  |controller|
   +---------+      +----------+       +----------+  +----------+
        \_____________ foundation (config / state) _____________/

   force.controller depends on: a MotionController (move_l, encode_command,
   send path) + a ForceSensor (read).  force.sensor depends on: state.RobotState.
```

Dependency summary: `config` and `state` depend on nothing else in the library;
`safety` depends on `config`; `connection` and `script_sender` depend on
`config`; `motion` depends on `config` + `connection` + `safety` + `state`;
`robot` depends on `config` + `connection` + `motion` + `script_sender` +
`state`; `force.sensor` depends on `state`; `force.controller` depends on
`config` + `force.sensor` (and a motion-controller-shaped object).

## Command protocol (PC -> daemon)

Fixed **10-float ASCII tuple** sent as a string and read by the daemon with
`socket_read_ascii_float(10, ...)`:

```
"(cmd, a0, a1, a2, a3, a4, a5, accel, vel, time)"
```

| cmd | name | a0 | a1 | a2 | a3 | a4 | a5 | accel | vel | time |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | moveL | x (m) | y (m) | z (m) | rx (rad) | ry (rad) | rz (rad) | m/s^2 | m/s | s |
| 1 | moveJ | j0 (rad) | j1 (rad) | j2 (rad) | j3 (rad) | j4 (rad) | j5 (rad) | rad/s^2 | rad/s | s |
| 2 | stop | decel (m/s^2) | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 3 | home | 0 | 0 | 0 | 0 | 0 | 0 | accel | vel | s |
| 4 | force | dx (unit) | dy (unit) | dz (unit) | target force (N) | stiffness | max travel (m) | accel | vel | s |

Notes:

* For **moveL** (`cmd=0`) the pose `a0..a5` is in the **UR base frame** (the
  motion layer has already applied `world_to_ur`).
* For **home** (`cmd=3`) the payload is all zeros; the daemon substitutes its
  configured home pose (`config.home_pose`, UR base frame).
* For **force** (`cmd=4`) `(dx, dy, dz)` is a *unit* approach direction in the UR
  base frame.

## State stream (daemon -> PC)

ASCII frames, each terminated by `+`. Several frames may arrive in one read;
parsers always use the **last complete** frame.

```
p[x,y,z,rx,ry,rz]_p[vx,vy,vz,wx,wy,wz]_[j0,j1,j2,j3,j4,j5]_p[fx,fy,fz,tx,ty,tz]+
```

| Position | Group | Contents | Units (UR base frame) |
|---|---|---|---|
| 1 | `p[...]` | TCP pose `[x, y, z, rx, ry, rz]` | m, rad |
| 2 | `p[...]` | TCP speed `[vx, vy, vz, wx, wy, wz]` | m/s, rad/s |
| 3 | `[...]` | joint angles `[j0, j1, j2, j3, j4, j5]` | rad |
| 4 | `p[...]` | TCP wrench `[fx, fy, fz, tx, ty, tz]` | N, Nm |

`parse_state` (see [`state`](state.md)) splits on `+` (take the last complete
frame), then on `_` into the four groups above, and parses each into six floats,
returning a `RobotState` stamped with `timestamp = time.time()`.

## Quick start

```python
from ur5e_control.config import RobotConfig
from ur5e_control.robot import UR5eRobot

with UR5eRobot(RobotConfig()) as robot:               # uploads daemon, starts stream
    robot.move_l([0.1, 0.3, 0.2, 0.0, -3.14, 0.0])    # world frame, m/rad
    robot.move_j([0.0, -1.57, 1.57, 0.0, 1.57, 0.0])  # joints, rad
    state = robot.get_state()                         # RobotState, UR base frame
    print(state.tcp_pose, state.wrench)
    robot.home()
```
