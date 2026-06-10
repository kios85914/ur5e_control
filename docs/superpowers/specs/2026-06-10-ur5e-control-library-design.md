# UR5e Pure-Control Library — Design Spec

**Date:** 2026-06-10
**Status:** Approved (design), pending spec review
**Author:** Po-Yen Wu (with Claude)

## 1. Goal

Refactor the existing UR5e codebase from an RL-specific, X/Y-restricted control
script into a clean, complete, extensible **pure robot-control library**. No
reinforcement learning. The primary concrete deliverable is a *complete*
move-end-effector capability (full 6-DOF Cartesian + joint-space), built on the
existing URScript-over-socket transport. The library must be ready to receive a
**Robotiq FT 300** force/torque sensor for force / impedance control later.

Deliverables:
1. A clean Python control package (`ur5e_control/`).
2. A generalized URScript motion daemon (full 6-DOF, not X/Y-locked).
3. A force-control subsystem (interface + control law + mock), targeting the
   Robotiq FT 300, marked *pending hardware validation*.
4. API-reference documentation (markdown).
5. A polished HTML manual generated with the `ui-ux-pro-max` skill.

## 2. Key decisions (confirmed with user)

| Decision | Choice | Implication |
|---|---|---|
| Control backend | **Keep custom URScript socket** (not `ur_rtde`) | Reuse the proven transport; generalize, don't replace. Force control via URScript `force_mode`. |
| Move-EE scope | **Cartesian + Joint** | `move_l` (full 6-DOF, abs + relative) and `move_j`. Realtime servo streaming is out of scope for now. |
| FT signal source | **Robotiq FT 300 (primary)** | Force-control subsystem targets the FT 300, not the UR built-in sensor. Behind a `ForceSensor` abstraction. |
| Legacy RL/sim/vision code | **Move to `legacy/`** | `env2.py`, `model.py`, `root_sac.py`, `main.py`, `camera.py`, `realsense_location_test.py`, old `ur_env_xy.py`, `ur_script_sender_xy.py`, `move_ee_xy.script`, `parameter.yaml`, `UserDefinedSettings.py` moved untouched. |
| Version control | **git initialized** | Baseline committed as a restore point before refactor. |

## 3. Background — what exists today

- **Transport:** raw URScript over TCP. `ur_script_sender_xy.py` uploads
  `move_ee_xy.script` to the controller (`192.168.0.135:30001`). The daemon
  opens a socket back to the PC (`192.168.0.120:30002`), receives action strings
  `(x, y, z, stop_flag)`, and streams state at ~500 Hz.
- **Motion:** hardcoded `movel(p[x,y,z, 0, -3.14, 0], a=0.1, v=0.1, t=2.0)`.
- **X/Y restriction:** Z fixed (0.115 m), rotation fixed (`ry = -π`),
  `action_dim = 2`, Python clamps only X/Y. `main.py` appends Z manually.
- **Force:** the URScript `force` field is a hardcoded zero vector — there is
  **no real force reading today** and **no Robotiq code anywhere**.
- **Frame hack:** `action[:2] = -action[:2]` hidden world↔UR negation.
- **Not-control code to archive:** Isaac Gym sim (`env2.py`, ~1100 lines), SAC
  RL (`model.py`, `root_sac.py`, `main.py`, missing `SAC.*` modules), RealSense
  vision (`camera.py`).

## 4. Target architecture

```
ur5e_control/
  __init__.py
  config.py        # RobotConfig dataclass: IPs/ports, default speed/accel,
                   #   workspace limits, frame/units, convergence tolerance,
                   #   explicit world<->UR frame transform (replaces the hidden negation)
  connection.py    # RobotConnection: socket lifecycle, thread-safe send/recv,
                   #   reconnect, clean shutdown
  script_sender.py # uploads the URScript daemon to the controller (:30001)
  state.py         # RobotState dataclass + parser for the streamed state string
  motion.py        # MotionController: builds/sends move commands, blocking
                   #   convergence check
  safety.py        # workspace + speed limit checks, command validation, e-stop
  robot.py         # UR5eRobot facade (the public API), context-manager support
  force/
    __init__.py
    sensor.py      # ForceSensor ABC, RobotiqFT300, MockForceSensor
    controller.py  # ForceController: move-until-force then hold (impedance)
  urscript/
    motion_daemon.script   # generalized daemon: 6-DOF moveL + moveJ + state
                           #   stream + force-mode dispatch
  examples/
    move_example.py
    force_control_example.py   # pending hardware
legacy/            # all archived RL/sim/vision/old-control files, untouched
docs/
  api/             # markdown API reference (one file per module)
  manual/          # HTML manual (ui-ux-pro-max output)
  superpowers/specs/   # this spec
```

### 4.1 Module responsibilities (one purpose each)

- **`config.py`** — `RobotConfig` dataclass. All tunables in one typed place:
  controller IP/port, PC host/port, default speed/accel/motion-time, workspace
  bounds (per-axis min/max), joint limits, convergence tolerance, units, and an
  explicit `world_to_ur` / `ur_to_world` transform. No hidden negation.
- **`connection.py`** — `RobotConnection`. Owns the listening socket, accepts the
  daemon callback, runs thread-safe send (action) and receive (state) loops with
  locks, handles reconnect and clean shutdown. Knows nothing about motion
  semantics.
- **`script_sender.py`** — uploads `urscript/motion_daemon.script` to the
  controller. Cleaned replacement for `ur_script_sender_xy.py`.
- **`state.py`** — `RobotState` dataclass `(tcp_pose, tcp_speed, joints, wrench,
  timestamp)` and a pure parser turning the streamed string into a `RobotState`.
- **`motion.py`** — `MotionController`. Encodes the command protocol (§4.2),
  sends moves, and implements blocking completion via position-error convergence
  (parameterized tolerance, replacing the magic `0.0001`).
- **`safety.py`** — validates every command against workspace/joint/speed limits
  before it is sent; provides emergency-stop and home. Raises on violation.
- **`robot.py`** — `UR5eRobot` facade. The single public entry point users
  import. Composes the above. Supports `with UR5eRobot(config) as r:`.
- **`force/`** — force-control subsystem (§4.4).

### 4.2 Command protocol (generalized, still socket strings)

Same transport, richer payload. Fixed-width float array with a leading command
code, sent as the existing ASCII-float format the daemon already parses:

```
[cmd, a0, a1, a2, a3, a4, a5, accel, vel, time]

cmd = 0  moveL   -> a0..a5 = pose p[x, y, z, rx, ry, rz]
cmd = 1  moveJ   -> a0..a5 = joint angles [j0..j5]
cmd = 2  stop    -> stopl(deceleration)
cmd = 3  home    -> move to configured home pose
cmd = 4  force   -> force_mode parameters (direction, target wrench, compliance)
```

The daemon dispatches on `cmd`. Adding force control later is just enabling
`cmd = 4` — no protocol change.

### 4.3 Public API (motion)

`UR5eRobot`:
- `connect()` / `disconnect()` (and context manager)
- `move_l(pose, speed=None, accel=None, blocking=True, relative=False)`
  — full 6-DOF linear move; `pose` is `[x, y, z, rx, ry, rz]` (meters, radians,
  base frame); `relative=True` adds to current pose.
- `move_j(joints, speed=None, accel=None, blocking=True)` — joint-space move.
- `get_state() -> RobotState`
- `stop()` — controlled stop.
- `home()` — move to configured home pose.

Units & frames (documented everywhere): **meters, radians, UR base frame**.
World↔UR conversion is explicit in `config.py`, not hidden.

### 4.4 Force-control subsystem (Robotiq FT 300, forward-looking)

- **`ForceSensor` (ABC)** — `read() -> Wrench` (fx, fy, fz, tx, ty, tz), `zero()`.
- **`RobotiqFT300(ForceSensor)`** — concrete impl for the FT 300. The exact read
  path (via the daemon's streamed force if the Robotiq URCap feeds URScript, vs.
  direct PC-side serial/USB driver) is **to be confirmed against hardware**; the
  abstraction isolates this choice.
- **`MockForceSensor(ForceSensor)`** — scriptable wrench source so the control
  law can be unit-tested with no hardware.
- **`ForceController`** — implements the target behavior: *move along a given
  direction until |force| reaches a target N, then maintain contact via
  impedance/compliance*. Built on URScript `force_mode` (`cmd = 4`) and/or a
  Python loop over streamed force.

This subsystem ships with interface + control law + mock + example. It is
explicitly **marked pending hardware validation**; on-robot tuning happens when
the FT 300 is mounted.

## 5. URScript daemon (`motion_daemon.script`)

Generalized from `move_ee_xy.script`:
- Full 6-DOF `movel` driven by the streamed pose (no hardcoded Z / rotation).
- `movej` support.
- State stream: TCP pose, TCP speed, joint positions, and **force** (real value
  once FT 300 is integrated; until then the controller's wrist reading or zeros).
- `cmd`-based dispatch matching §4.2, including a `force_mode` branch.
- Preserve the proven connection/threading structure (callback to PC, send/recv
  threads) to minimize risk on the user's setup.

## 6. Documentation deliverables

- **`docs/api/`** — markdown API reference, one file per module. Each public
  class/function documents: purpose, signature, parameters **with units and
  coordinate frame**, return value, exceptions, and a short usage example.
- **`docs/manual/`** — HTML manual produced with **`ui-ux-pro-max`**. Sections:
  Quick Start, Architecture overview (with diagram), full API reference, runnable
  examples, **Safety**, Force-control guide, Troubleshooting. Goal: a new user
  gets moving quickly.

## 7. Safety requirements

This drives a real robot arm. The library must:
- Validate every commanded pose/joint target against configured workspace/joint
  limits before sending; raise on violation.
- Enforce a configurable max speed/accel.
- Provide an always-available `stop()` (emergency) and `home()`.
- Default to conservative speed/accel.
- The manual must include a prominent Safety section.

## 8. Verification approach (honest scope)

**Cannot test on the real UR5e** — moving a physical arm unsupervised is unsafe
and out of scope. Verification at code level only:
- Imports / structure / lint clean.
- Mock-based unit tests for: state-string parsing, command-protocol encoding,
  safety-limit validation, and the force-control law (via `MockForceSensor`).
- A dry-run / no-connect mode that builds and logs commands without sending.

**On-robot validation (motion correctness, FT 300 integration, force tuning) is
the user's step.** This is stated in the manual.

## 9. Implementation via multi-agent workflow

Implementation fans out with the Workflow tool (user explicitly opted into
multi-agent collaboration):
- Parallel build agents: (a) `config`+`state`+`safety`, (b) `connection`,
  (c) `script_sender`+`motion_daemon.script`, (d) `motion`+`robot`,
  (e) `force/` subsystem, (f) examples + mock unit tests.
- Then: API-doc markdown agent(s), then a single agent runs `ui-ux-pro-max` for
  the HTML manual.
- Then a review + verification pass (imports, tests, interface consistency).
Interfaces (`RobotConfig`, `RobotState`, command protocol, `ForceSensor`) are
fixed by this spec so parallel agents stay compatible.

## 10. Out of scope

- Reinforcement learning (archived to `legacy/`).
- Isaac Gym simulation (archived).
- RealSense vision / object detection (archived).
- `ur_rtde` migration.
- Realtime servo/speed streaming control.
- On-robot testing and FT 300 hardware bring-up.

## 11. Open items to confirm at hardware time

- Exact Robotiq FT 300 read path (URCap-fed URScript vs. PC-side driver).
- Whether `force_mode` with the FT 300 needs the Robotiq URCap, or uses the
  controller's force interface.
- Home pose and final workspace limits for the new project's setup.
