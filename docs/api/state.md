# `ur5e_control.state`

Robot state model and parser. The daemon streams the robot state back to the PC
as ASCII strings; this module models a single snapshot (`RobotState`) and parses
raw frames (`parse_state`). All quantities are in the **UR base frame**, meters
and radians (velocities m/s, rad/s; forces N, Nm).

## State-stream frame format

```
p[x,y,z,rx,ry,rz]_p[vx,vy,vz,wx,wy,wz]_[j0,j1,j2,j3,j4,j5]_p[fx,fy,fz,tx,ty,tz]+
```

Groups, in order:

| Group | Prefix | Contents | Units |
|---|---|---|---|
| TCP pose | `p[...]` | `[x, y, z, rx, ry, rz]` | m, rad |
| TCP speed | `p[...]` | `[vx, vy, vz, wx, wy, wz]` | m/s, rad/s |
| joint angles | `[...]` | `[j0, j1, j2, j3, j4, j5]` | rad |
| TCP wrench | `p[...]` | `[fx, fy, fz, tx, ty, tz]` | N, Nm |

Frames are delimited by `+`. Several frames may arrive concatenated in one read;
`parse_state` always uses the **last complete** frame.

---

## `class RobotState`

A single snapshot of the UR5e robot state. A dataclass. **Field order is part of
the locked interface and must not change.**

### Fields (in order)

| Field | Type | Units / meaning |
|---|---|---|
| `tcp_pose` | `list[float]` | TCP pose `[x, y, z, rx, ry, rz]` (m, rad). |
| `tcp_speed` | `list[float]` | TCP speed `[vx, vy, vz, wx, wy, wz]` (m/s, rad/s). |
| `joints` | `list[float]` | Joint angles `[j0..j5]` (rad). |
| `wrench` | `list[float]` | TCP wrench `[fx, fy, fz, tx, ty, tz]` (N, Nm). |
| `timestamp` | `float` | `time.time()` captured when the frame was parsed (s). |

All quantities are expressed in the **UR base frame**.

---

## `parse_state(raw: str) -> RobotState`

**Purpose.** Parse a raw daemon state string into a `RobotState`.

* **Parameters.** `raw` — one or more concatenated state frames in the format
  above.
* **Returns.** A `RobotState` stamped with `timestamp = time.time()`. Parsed
  poses, speeds, joints and wrenches are in the **UR base frame** (m/rad).
* **Behavior.** The input may contain several `+`-delimited frames; the **last
  complete** frame (last one terminated by `+`) is parsed. A trailing partial
  frame (text after the final `+`) is ignored. The frame is split on `_` into
  four groups (`p[...]`/`p[...]`/`[...]`/`p[...]`); each group is parsed into
  exactly six floats.
* **Exceptions.**
  * `ValueError` if `raw` is not a `str`.
  * `ValueError` if there is no complete frame (no `+` terminator, or no
    non-empty complete frame).
  * `ValueError` if the last complete frame is malformed: not exactly 4
    underscore-delimited groups, a missing `p` prefix / brackets, a wrong
    element count (not 6), or a non-numeric value.

### Usage example

```python
from ur5e_control.state import parse_state, RobotState

raw = (
    "p[0.1,0.2,0.3,0.0,-3.14,0.0]"
    "_p[0.0,0.0,0.0,0.0,0.0,0.0]"
    "_[0.0,-1.57,1.57,0.0,1.57,0.0]"
    "_p[1.0,2.0,3.0,0.1,0.2,0.3]+"
)
state: RobotState = parse_state(raw)
print(state.tcp_pose)   # [0.1, 0.2, 0.3, 0.0, -3.14, 0.0]  (UR base frame, m/rad)
print(state.wrench)     # [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]    (N, Nm)
print(state.timestamp)  # e.g. 1749513600.123  (seconds)

# Several frames concatenated -> last complete one is used:
parse_state("p[...]..._...+p[9,9,9,9,9,9]_p[...]_[...]_p[...]+partial")
```
