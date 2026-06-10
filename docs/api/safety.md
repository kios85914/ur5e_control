# `ur5e_control.safety`

Pure safety validators for UR5e motion commands. These functions enforce the
limits declared on a `RobotConfig` before any command is sent. There is **no
socket logic** here — they only inspect numbers and raise typed exceptions on
violations so the motion layer can fail fast.

## Conventions

* Poses are `[x, y, z, rx, ry, rz]` in **meters** and **radians**, **UR base
  frame**.
* Joint targets are `[j0, j1, j2, j3, j4, j5]` in **radians**.
* Speeds are scalars in **m/s** (moveL) or **rad/s** (moveJ); the cap is
  `RobotConfig.max_speed`.

---

## Exception hierarchy

| Class | Base | Raised when |
|---|---|---|
| `SafetyViolation` | `Exception` | Base class — catch this to catch any violation below. |
| `WorkspaceViolation` | `SafetyViolation` | Pose outside the workspace, or not a well-formed 6-element list. |
| `SpeedViolation` | `SafetyViolation` | Requested speed is invalid (non-positive). |
| `JointLimitViolation` | `SafetyViolation` | Joint targets are the wrong count or out of range. |

---

## `check_pose_in_workspace(pose: Sequence[float], config: RobotConfig) -> bool`

**Purpose.** Validate that a Cartesian pose lies within the configured workspace.

* **Parameters.**
  * `pose` — 6-element sequence `[x, y, z, rx, ry, rz]` (m, rad), UR base frame.
  * `config` — supplies `workspace_limits`.
* **Returns.** `True` if the pose is within bounds.
* **Behavior.** Only the translational components `x, y, z` (m) are
  bounds-checked against `config.workspace_limits`; the rotation components are
  not constrained here. Bounds are **inclusive**.
* **Exceptions.** `WorkspaceViolation` if `pose` is not length 6, or any of
  x/y/z falls outside its `(min, max)` limit (the message names the offending
  axis).

## `check_joints(joints: Sequence[float], config: RobotConfig) -> bool`

**Purpose.** Validate that joint targets are 6 angles within their limits.

* **Parameters.**
  * `joints` — 6-element sequence `[j0..j5]` (rad).
  * `config` — supplies `joint_limits` (a 6-tuple of `(min, max)` pairs in rad).
* **Returns.** `True` if all six joints are within bounds.
* **Behavior.** Bounds are **inclusive**.
* **Exceptions.** `JointLimitViolation` if `joints` is not length 6, or any
  joint is out of range (the message names the offending joint index).

## `clamp_speed(speed: float, config: RobotConfig) -> float`

**Purpose.** Clamp a requested speed to the configured safety cap.

* **Parameters.**
  * `speed` — requested speed in m/s (moveL) or rad/s (moveJ). Must be > 0.
  * `config` — supplies `max_speed` (m/s).
* **Returns.** The speed, capped at `config.max_speed`. Speeds at or below the
  cap are returned unchanged.
* **Exceptions.** `SpeedViolation` if `speed` is not strictly positive (rejected
  as a typed error rather than a silent clamp, since it would never converge).

### Usage example

```python
from ur5e_control.config import RobotConfig
from ur5e_control.safety import (
    check_pose_in_workspace, check_joints, clamp_speed,
    WorkspaceViolation, SpeedViolation, SafetyViolation,
)

cfg = RobotConfig()

check_pose_in_workspace([-0.1, 0.3, 0.2, 0.0, -3.14, 0.0], cfg)   # True
check_joints([0.0, -1.57, 1.57, 0.0, 1.57, 0.0], cfg)             # True
clamp_speed(0.5, cfg)                                             # 0.25 (== max_speed)

try:
    check_pose_in_workspace([10.0, 0.3, 0.2, 0, 0, 0], cfg)
except WorkspaceViolation as exc:
    print("rejected:", exc)

# Catch any safety problem with the base class:
try:
    clamp_speed(-1.0, cfg)
except SafetyViolation:
    ...
```
