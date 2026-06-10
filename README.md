# ur5e_control

A clean, dependency-free Python library for controlling a Universal Robots **UR5e**
over the URScript socket transport. Full 6-DOF Cartesian and joint motion, with a
forward-looking **Robotiq FT 300** force-control subsystem.

> This replaces the previous RL/simulation codebase, which is preserved untouched
> under [`legacy/`](legacy/) (SAC, Isaac Gym sim, RealSense vision).

## Install / test

Runtime needs only the Python standard library (Python 3.10+). For the test suite:

```bash
uv venv --python 3.10 .venv
uv pip install --python .venv/bin/python pytest
.venv/bin/python -m pytest -q          # 129 passed
```

## Quick start

```python
from ur5e_control import UR5eRobot, RobotConfig

with UR5eRobot(RobotConfig()) as robot:        # connects on enter, disconnects on exit
    robot.move_l([0.1, 0.3, 0.2, 0.0, -3.14, 0.0])   # world frame, meters / radians
    state = robot.get_state()                  # RobotState (UR base frame)
    print(state.tcp_pose, state.wrench)
    robot.home()
```

`connect()` uploads the URScript daemon to the controller and starts the PC-side
socket so the daemon's state stream flows back. Everything is **meters, radians,
UR base frame**; world↔UR conversion is explicit in `RobotConfig`.

## Layout

```
ur5e_control/
  config.py        RobotConfig + explicit world<->UR frame transforms
  state.py         RobotState dataclass + parse_state()
  safety.py        workspace / joint / speed validators + typed exceptions
  connection.py    thread-safe socket transport (send commands, receive state, dry_run)
  script_sender.py uploads the URScript daemon to the controller
  motion.py        MotionController: byte-exact command encoding, safety, convergence
  robot.py         UR5eRobot facade (the public API)
  force/           ForceSensor (Robotiq FT 300 / Mock) + ForceController  [forward-looking]
  urscript/motion_daemon.script   generalized 6-DOF daemon (moveL/moveJ/stop/home/force)
  examples/        runnable dry-run / mock examples
tests/             pytest suite (parsing, encoding, safety, control law via mocks)
docs/
  api/             markdown API reference, one file per module
  manual/index.html   polished HTML user manual  <-- open this to get started
  superpowers/specs/  design spec    superpowers/plans/  implementation plan
legacy/            archived RL / simulation / vision code
```

## Documentation

- **HTML manual:** open [`docs/manual/index.html`](docs/manual/index.html) in a browser.
- **API reference:** [`docs/api/`](docs/api/) (markdown, one file per module).

## ⚠️ Safety & verification scope

This drives a **real robot arm**. Read the Safety section of the manual before
connecting to hardware. The library is verified only at the **code level**
(parsing, command encoding, safety validation, and the force-control law via
mocks — 129 tests). **Motion correctness on real hardware, the URScript daemon
upload, and Robotiq FT 300 integration are not verified here** and must be
validated by the operator on the actual UR5e and cell. The daemon's force-mode
branch is disabled by default (`FORCE_MODE_ENABLED = False`) until validated.
