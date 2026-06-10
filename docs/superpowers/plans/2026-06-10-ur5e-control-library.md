# UR5e Pure-Control Library Implementation Plan

> **For agentic workers:** This plan is executed by a multi-agent Workflow. Each
> task is a self-contained component. The **Locked Interfaces** section is the
> contract every agent MUST honor verbatim so parallel work stays compatible.
> Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor the UR5e codebase into a clean pure-control library with full
6-DOF Cartesian + joint motion over the existing URScript socket transport, plus
a forward-looking Robotiq FT 300 force-control subsystem, API docs, and an HTML
manual.

**Architecture:** A Python package `ur5e_control/` with a layered design
(config/state → connection → motion → robot facade → force). The proven URScript
socket transport is kept but generalized to a `[cmd, a0..a5, accel, vel, time]`
command protocol. Legacy RL/sim/vision code is archived under `legacy/`.

**Tech Stack:** Python 3.10, stdlib `socket`/`threading`/`dataclasses`/`abc`,
`pytest` + `unittest.mock` for tests, URScript (daemon), `ui-ux-pro-max` (HTML
manual). No `ur_rtde`, no RL/sim/vision deps in the new package.

**Spec:** `docs/superpowers/specs/2026-06-10-ur5e-control-library-design.md`

---

## Locked Interfaces (the contract — do not deviate)

### Units & frames
Meters, radians, **UR base frame** everywhere. World↔UR conversion is explicit in
`RobotConfig` (no hidden negation).

### Command protocol (PC → daemon)
Fixed 10-float ASCII tuple, sent as a string `"(cmd, a0, a1, a2, a3, a4, a5, accel, vel, time)"`.
Daemon reads with `socket_read_ascii_float(10, ...)`.

| cmd | meaning | a0..a5 | accel/vel/time |
|----|---------|--------|----------------|
| 0 | moveL | pose `x,y,z,rx,ry,rz` (m, rad) | a (m/s²), v (m/s), t (s) |
| 1 | moveJ | joints `j0..j5` (rad) | a (rad/s²), v (rad/s), t (s) |
| 2 | stop | a0 = deceleration (m/s²); a1..a5 = 0 | unused (send 0) |
| 3 | home | all 0 (daemon uses configured home) | a, v, t |
| 4 | force | a0..a2 = direction unit vec; a3 = target force N; a4 = stiffness; a5 = max travel (m) | a, v, t |

### State stream (daemon → PC)
String: `p[x,y,z,rx,ry,rz]_p[vx,vy,vz,wx,wy,wz]_[j0,j1,j2,j3,j4,j5]_p[fx,fy,fz,tx,ty,tz]+`
(unchanged from the existing daemon format).

### `RobotState` (ur5e_control/state.py)
```python
@dataclass
class RobotState:
    tcp_pose: list[float]    # [x,y,z,rx,ry,rz]  m, rad, base frame
    tcp_speed: list[float]   # [vx,vy,vz,wx,wy,wz]
    joints: list[float]      # [j0..j5] rad
    wrench: list[float]      # [fx,fy,fz,tx,ty,tz] N, Nm
    timestamp: float         # time.time() when parsed
```

### `RobotConfig` (ur5e_control/config.py)
```python
@dataclass
class RobotConfig:
    controller_ip: str = "192.168.0.135"
    script_port: int = 30001
    pc_host: str = "192.168.0.120"
    state_port: int = 30002
    default_speed: float = 0.1        # m/s
    default_accel: float = 0.1        # m/s^2
    default_move_time: float = 2.0    # s
    convergence_tol: float = 1e-3     # m
    workspace_limits: dict = ...      # {"x": (min,max), "y": (min,max), "z": (min,max)}
    joint_limits: tuple = ...         # 6 x (min,max) rad
    max_speed: float = 0.25           # m/s safety cap
    home_pose: list[float] = ...      # [x,y,z,rx,ry,rz]
    def world_to_ur(self, pose: list[float]) -> list[float]: ...
    def ur_to_world(self, pose: list[float]) -> list[float]: ...
```
Defaults for `workspace_limits` mirror the legacy clamps: x (-0.40, 0.35),
y (0.25, 0.80), z (0.0, 0.40). `world_to_ur`/`ur_to_world` default to negating
x and y (preserving legacy behavior) but as explicit, overridable methods.

### `ForceSensor` ABC (ur5e_control/force/sensor.py)
```python
class ForceSensor(ABC):
    @abstractmethod
    def read(self) -> list[float]: ...   # [fx,fy,fz,tx,ty,tz]
    @abstractmethod
    def zero(self) -> None: ...
```

### `UR5eRobot` public API (ur5e_control/robot.py)
```python
class UR5eRobot:
    def __init__(self, config: RobotConfig = RobotConfig()): ...
    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def move_l(self, pose, speed=None, accel=None, blocking=True, relative=False) -> None: ...
    def move_j(self, joints, speed=None, accel=None, blocking=True) -> None: ...
    def get_state(self) -> RobotState: ...
    def stop(self) -> None: ...
    def home(self) -> None: ...
    def __enter__(self): ...   # returns self, calls connect()
    def __exit__(self, *exc): ...  # calls disconnect()
```

---

## Task 0: Scaffold package + archive legacy

**Files:**
- Create: `ur5e_control/__init__.py`, `ur5e_control/force/__init__.py`,
  `ur5e_control/urscript/`, `ur5e_control/examples/`, `tests/`, `docs/api/`,
  `docs/manual/`
- Move to `legacy/`: `env2.py`, `model.py`, `root_sac.py`, `main.py`,
  `camera.py`, `realsense_location_test.py`, `ur_env_xy.py`,
  `ur_script_sender_xy.py`, `move_ee_xy.script`, `parameter.yaml`,
  `UserDefinedSettings.py`
- Create: `pyproject.toml` (package metadata, pytest config), `requirements.txt`

- [ ] Move legacy files with `git mv` (preserves history).
- [ ] Create package dirs with `__init__.py`. `ur5e_control/__init__.py` exports
      `UR5eRobot`, `RobotConfig`, `RobotState`.
- [ ] `pyproject.toml`: package name `ur5e-control`, py3.10, pytest config
      (`testpaths = ["tests"]`).
- [ ] Commit: `chore: scaffold ur5e_control package, archive legacy code`.

**Acceptance:** `legacy/` holds all old files; new dirs exist; `pytest` collects 0
tests without import errors.

---

## Task 1: `config.py` — RobotConfig + frame transforms

**Files:** Create `ur5e_control/config.py`, `tests/test_config.py`

- [ ] **Test first** (`tests/test_config.py`): `world_to_ur` negates x,y and
      round-trips with `ur_to_world`; defaults match the locked values; `home_pose`
      has length 6.
- [ ] Run `pytest tests/test_config.py -v` → FAIL (no module).
- [ ] Implement `RobotConfig` exactly per Locked Interfaces. Frame transforms are
      pure functions on a 6-list (only x,y negated by default; z/rotations
      unchanged).
- [ ] Run tests → PASS. Commit.

**Acceptance:** round-trip transform is identity; defaults correct.

---

## Task 2: `state.py` — RobotState + parser

**Files:** Create `ur5e_control/state.py`, `tests/test_state.py`

- [ ] **Test first:** parse a known sample stream string (use the exact format in
      Locked Interfaces) and assert each field parses to the right floats; assert
      malformed input raises `ValueError`; assert the latest `+`-delimited frame is
      used when several arrive concatenated.
- [ ] Run → FAIL.
- [ ] Implement `RobotState` dataclass + `parse_state(raw: str) -> RobotState`.
      Strategy: split on `+`, take last complete frame, split on `_`, parse the
      `p[...]` / `[...]` groups into float lists, stamp `timestamp=time.time()`.
- [ ] Run → PASS. Commit.

**Acceptance:** parser handles the documented format and concatenated frames; bad
input raises.

---

## Task 3: `safety.py` — limit validation + e-stop helpers

**Files:** Create `ur5e_control/safety.py`, `tests/test_safety.py`

- [ ] **Test first:** `check_pose_in_workspace(pose, config)` returns True inside,
      raises `WorkspaceViolation` outside (test each of x/y/z bounds); `clamp_speed`
      caps at `config.max_speed`; `check_joints(joints, config)` validates 6 joints.
- [ ] Run → FAIL.
- [ ] Implement validators + custom exceptions (`WorkspaceViolation`,
      `SpeedViolation`, `JointLimitViolation`). Pure functions taking a
      `RobotConfig`. No socket logic here.
- [ ] Run → PASS. Commit.

**Acceptance:** every limit in `RobotConfig` is enforced; violations raise typed
exceptions.

---

## Task 4: `connection.py` — RobotConnection socket layer

**Files:** Create `ur5e_control/connection.py`, `tests/test_connection.py`

- [ ] **Test first (no real robot):** use a local loopback socket server fixture
      to verify `RobotConnection` binds `pc_host:state_port`, accepts a client,
      `send(msg)` writes bytes, the receive loop captures the latest state string
      thread-safely, and `close()` shuts cleanly. Mock where loopback is awkward.
- [ ] Run → FAIL.
- [ ] Implement `RobotConnection(config)`: `start()` (bind/listen/accept),
      `send(str)` (thread-safe, encodes ascii), `latest_state() -> str` (lock-
      guarded), background receive thread, `close()`. Reconnect on broken pipe.
      No motion semantics. Add a `dry_run` flag that skips real sockets and logs.
- [ ] Run → PASS. Commit.

**Acceptance:** loopback test passes; thread-safe access; clean shutdown; dry-run
mode logs without sockets.

---

## Task 5: `script_sender.py` — upload daemon to controller

**Files:** Create `ur5e_control/script_sender.py`, `tests/test_script_sender.py`

- [ ] **Test first:** `load_script(path)` reads the daemon file and returns bytes;
      `send_script(script_bytes, config, dry_run=True)` returns without connecting.
      Mock `socket` to assert it connects to `controller_ip:script_port` and sends
      the bytes in non-dry-run.
- [ ] Run → FAIL.
- [ ] Implement: read `ur5e_control/urscript/motion_daemon.script`, connect to
      `controller_ip:script_port`, send, close. `dry_run` skips the socket.
- [ ] Run → PASS. Commit.

**Acceptance:** uploads the daemon; dry-run + mock tests pass.

---

## Task 6: `motion_daemon.script` — generalized URScript daemon

**Files:** Create `ur5e_control/urscript/motion_daemon.script`

(No unit test — runs on the controller. Reviewed for correctness + protocol match.)

- [ ] Generalize `legacy/move_ee_xy.script`:
      - Connect back to `pc_host:state_port`, spawn send/recv threads (keep the
        proven structure).
      - `receive_action()` reads **10** floats.
      - Dispatch on `cmd`: 0 `movel(p[a0..a5], accel, vel, time)`; 1
        `movej([a0..a5], accel, vel, time)`; 2 `stopl(a0)`; 3 `movel(<home>, ...)`;
        4 `force_mode(...)` per the force-cmd encoding (guarded, for FT 300).
      - `send_state()` streams the exact documented format; include real force via
        `get_tcp_force()` (FT 300 once integrated; controller wrist meanwhile).
      - No hardcoded Z / rotation.
- [ ] Add a header comment documenting the protocol + ports.
- [ ] Commit.

**Acceptance:** daemon implements the locked command protocol and state format;
code review confirms 6-DOF moveL/moveJ and the force_mode branch.

---

## Task 7: `motion.py` — MotionController

**Files:** Create `ur5e_control/motion.py`, `tests/test_motion.py`
**Depends on:** Tasks 1–4.

- [ ] **Test first:** `encode_command(cmd, payload, accel, vel, time)` produces the
      exact `"(...)"` string for each cmd (assert byte-for-byte); `move_l` calls
      `connection.send` with the moveL-encoded string; blocking convergence loop
      exits when `|tcp_pose - target| < convergence_tol` (feed states via a fake
      connection). Use a mock `RobotConnection`.
- [ ] Run → FAIL.
- [ ] Implement `MotionController(connection, config)`: `encode_command`,
      `move_l`, `move_j`, `stop`, `home`; blocking uses `parse_state` on
      `latest_state()` and compares to target within `convergence_tol` (replaces
      the magic 0.0001); applies frame transform + safety checks before sending.
- [ ] Run → PASS. Commit.

**Acceptance:** command encoding matches the protocol exactly; safety checks run
before send; blocking convergence works against fake states.

---

## Task 8: `robot.py` — UR5eRobot facade

**Files:** Create `ur5e_control/robot.py`, `tests/test_robot.py`
**Depends on:** Tasks 1–5, 7.

- [ ] **Test first:** with mocked `RobotConnection`/`script_sender`,
      `UR5eRobot(config)` as a context manager calls connect on enter / disconnect
      on exit; `move_l`/`move_j` delegate to `MotionController`; `get_state` returns
      a `RobotState`; `home`/`stop` send the right cmd.
- [ ] Run → FAIL.
- [ ] Implement the facade per Locked Interfaces, composing
      script_sender + connection + motion. `connect()` uploads the daemon then
      starts the connection. Defaults pulled from `config`.
- [ ] Run → PASS. Commit.

**Acceptance:** public API matches the contract; context manager works; delegation
verified with mocks.

---

## Task 9: `force/sensor.py` — ForceSensor + RobotiqFT300 + Mock

**Files:** Create `ur5e_control/force/sensor.py`, `tests/test_force_sensor.py`
**Depends on:** Task 2 (state) for the streamed-force path.

- [ ] **Test first:** `MockForceSensor` returns scripted wrenches in order and
      supports `zero()`; `RobotiqFT300.read()` (streamed-force backend) extracts the
      wrench from a `RobotState`/state string via an injected provider.
- [ ] Run → FAIL.
- [ ] Implement `ForceSensor` ABC; `MockForceSensor(sequence)`; `RobotiqFT300`
      taking a `state_provider` callable (returns latest `RobotState`) so the read
      path (URCap-fed stream now, PC-side driver later) is swappable. Add a
      module docstring: **pending hardware validation**, note the open read-path
      question from spec §11.
- [ ] Run → PASS. Commit.

**Acceptance:** mock + streamed-force read tested; backend is swappable; hardware
caveat documented.

---

## Task 10: `force/controller.py` — ForceController (impedance)

**Files:** Create `ur5e_control/force/controller.py`, `tests/test_force_controller.py`
**Depends on:** Tasks 7, 9.

- [ ] **Test first (mock sensor + mock motion):** `approach_until_force(direction,
      target_n, ...)` issues motion along `direction` and **stops/holds** once the
      `MockForceSensor` reports |force| ≥ target; verify it does not overshoot past
      the scripted contact step; verify it emits the cmd=4 force payload with the
      correct direction/target encoding.
- [ ] Run → FAIL.
- [ ] Implement `ForceController(motion, sensor, config)`:
      `approach_until_force(direction, target_n, stiffness, max_travel)` —
      move along `direction`, monitor `sensor.read()`, transition to
      `force_mode` (cmd=4) hold when target reached. Pure control law; testable
      with mocks. Docstring: **pending hardware validation / tuning**.
- [ ] Run → PASS. Commit.

**Acceptance:** the move-until-force-then-hold law passes mock tests; force payload
encoding matches the protocol.

---

## Task 11: examples

**Files:** Create `ur5e_control/examples/move_example.py`,
`ur5e_control/examples/force_control_example.py`

- [ ] `move_example.py`: connect, `move_l` to a pose, read state, `home`,
      disconnect — using a `RobotConfig`. Runnable with `dry_run` for safe preview.
- [ ] `force_control_example.py`: `ForceController.approach_until_force` demo with
      `MockForceSensor`; header notes it needs the FT 300 for real use.
- [ ] Commit.

**Acceptance:** both run under dry-run/mock without a robot.

---

## Task 12: API docs (markdown)

**Files:** Create `docs/api/<module>.md` for each public module.

- [ ] One file per module: `config`, `state`, `safety`, `connection`,
      `script_sender`, `motion`, `robot`, `force.sensor`, `force.controller`.
      Each documents purpose, every public class/function signature, parameters
      **with units and frame**, returns, exceptions, and a short example.
- [ ] `docs/api/index.md` links them and shows the architecture/layer diagram.
- [ ] Commit.

**Acceptance:** every public symbol from the Locked Interfaces is documented with
units/frames.

---

## Task 13: HTML manual (ui-ux-pro-max)

**Files:** Create `docs/manual/index.html` (+ assets).

- [ ] Use the `ui-ux-pro-max` skill to produce a polished, navigable manual:
      Quick Start, Architecture (diagram), API Reference (from `docs/api/`),
      Examples, **Safety** (prominent), Force-control guide, Troubleshooting.
- [ ] Commit.

**Acceptance:** a new user can open `index.html` and get from install → first move
quickly; Safety section is prominent.

---

## Task 14: Final verification pass

- [ ] `pytest -q` → all green; report counts.
- [ ] Import smoke test: `python -c "from ur5e_control import UR5eRobot, RobotConfig, RobotState"`.
- [ ] Lint/structure check; confirm command-encoding tests match the daemon's
      documented parsing; confirm no `legacy/` import leaks into `ur5e_control/`.
- [ ] Commit any fixes. Write a short `README` update for the new package.

**Acceptance:** tests pass, imports clean, interfaces consistent across modules.
On-robot validation explicitly deferred to the user.

---

## Self-Review notes
- **Spec coverage:** transport/protocol (T6,T7), 6-DOF + joint move (T7,T8),
  config/frames (T1), state (T2), safety (T3), force subsystem (T9,T10), legacy
  archive (T0), API docs (T12), HTML manual (T13), verification (T14) — all spec
  sections mapped.
- **Type consistency:** `RobotState`, `RobotConfig`, `ForceSensor`, and the 10-
  float command protocol are defined once in Locked Interfaces and referenced by
  all tasks.
- **Honest scope:** no task claims on-robot validation; force subsystem flagged
  pending hardware throughout.
