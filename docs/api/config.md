# `ur5e_control.config`

Configuration and frame transforms for the UR5e control library. `RobotConfig`
is the single source of truth for network endpoints, motion defaults, safety
limits, and the world<->UR-base frame conversion. **There is no hidden negation
anywhere else in the library** — all world<->UR coordinate conversion goes
through `RobotConfig.world_to_ur` and `RobotConfig.ur_to_world`.

## Units and frames

* Positions in **meters**, rotations in **radians**.
* A *pose* is `[x, y, z, rx, ry, rz]`, where `(rx, ry, rz)` is the axis-angle
  (rotation-vector) orientation.
* *Joints* are `[j0, j1, j2, j3, j4, j5]` in radians.
* Geometry stored in this config (`home_pose`, `workspace_limits`,
  `joint_limits`) is expressed in the **UR base frame**. By default the world
  frame equals the UR base frame (`world_to_ur`/`ur_to_world` are identity);
  override those methods if your world frame differs.

---

## `class RobotConfig`

Network, motion, and safety configuration for a UR5e robot. A dataclass; every
field has a default, so `RobotConfig()` is fully usable out of the box.

### Fields (with defaults, in declaration order)

| Field | Type | Default | Units / meaning |
|---|---|---|---|
| `controller_ip` | `str` | `"192.168.0.137"` | IP of the UR controller (daemon host). |
| `script_port` | `int` | `30001` | TCP port the controller listens on for URScript. |
| `pc_host` | `str` | `"192.168.0.120"` | IP of this PC, advertised to the daemon for the state-stream callback. |
| `state_port` | `int` | `30002` | TCP port on this PC that receives the daemon state stream. |
| `default_speed` | `float` | `0.1` | Default Cartesian/joint speed (m/s or rad/s). |
| `default_accel` | `float` | `0.1` | Default Cartesian/joint acceleration (m/s^2 or rad/s^2). |
| `default_move_time` | `float` | `2.0` | Default blend/move duration (s). |
| `convergence_tol` | `float` | `1e-3` | Tolerance (m/rad) for deciding a blocking move reached its target. |
| `workspace_limits` | `Dict[str, Tuple[float, float]]` | `{"x": (-0.40, 0.40), "y": (-0.565, -0.265), "z": (-0.10, 0.40)}` | Cartesian clamps in the UR base frame, per axis (m). |
| `joint_limits` | `Tuple[Tuple[float, float], ...]` | six `(-6.283185, 6.283185)` pairs | Per-joint `(min, max)` limits (rad). |
| `max_speed` | `float` | `0.25` | Hard upper bound on commanded speed (m/s or rad/s). |
| `home_pose` | `List[float]` | `[0.0, -0.35, 0.25, 0.0, -3.14, 0.0]` | Home pose `[x, y, z, rx, ry, rz]` (m, rad) in UR base frame. |

The mutable defaults (`workspace_limits`, `joint_limits`, `home_pose`) are
created by `default_factory`, so each instance owns an independent copy.

### `world_to_ur(self, pose: List[float]) -> List[float]`

**Purpose.** Convert a pose from the world frame to the UR base frame.

* **Parameters.** `pose` — world-frame pose `[x, y, z, rx, ry, rz]` in
  meters/radians.
* **Returns.** A new UR-base-frame pose `[x, y, z, rx, ry, rz]` (m, rad).
* **Behavior.** Negates only the x and y position axes; z and the rotation
  vector `(rx, ry, rz)` pass through unchanged. The input list is not mutated.
* **Exceptions.** None declared (a `pose` not of length 6 raises `ValueError`
  on unpacking).

### `ur_to_world(self, pose: List[float]) -> List[float]`

**Purpose.** Convert a pose from the UR base frame to the world frame. Inverse
of `world_to_ur`.

* **Parameters.** `pose` — UR-base-frame pose `[x, y, z, rx, ry, rz]` (m, rad).
* **Returns.** A new world-frame pose `[x, y, z, rx, ry, rz]` (m, rad).
* **Behavior.** Negates only x and y; z and rotations unchanged. Input not
  mutated. `ur_to_world(world_to_ur(p)) == p` (round-trips to identity).
* **Exceptions.** None declared.

### Usage example

```python
from ur5e_control.config import RobotConfig

cfg = RobotConfig()                       # all defaults
cfg = RobotConfig(controller_ip="192.168.0.200", max_speed=0.15)

ur = cfg.world_to_ur([0.1, 0.3, 0.2, 0.0, -3.14, 0.0])   # -> [-0.1, -0.3, 0.2, 0.0, -3.14, 0.0]
back = cfg.ur_to_world(ur)                                # -> [0.1, 0.3, 0.2, 0.0, -3.14, 0.0]
assert back == [0.1, 0.3, 0.2, 0.0, -3.14, 0.0]          # round-trips to identity
```
