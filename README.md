# ur5e_control

A clean, dependency-free Python library for controlling a Universal Robots **UR5e**
over the URScript socket transport. Full 6-DOF Cartesian and joint motion, with a
forward-looking **Robotiq FT 300** force-control subsystem.

> This replaces the previous RL/simulation codebase, which is preserved untouched
> under [`legacy/`](legacy/) (SAC, Isaac Gym sim, RealSense vision).

## Install / test

Runtime needs only the Python standard library (Python 3.10+). Set up the
environment with **conda**:

```bash
conda create -n ur5e python=3.10
conda activate ur5e
pip install pytest          # test suite only
python -m pytest -q         # 140 passed
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

## Control panel (web GUI)

A one-screen test harness for every feature — set the IP, connect, watch live
state, and jog the TCP / joints:

```bash
python -m ur5e_control.gui          # then open http://127.0.0.1:8080
```

It binds to localhost only and starts in **dry-run** (no robot needed — you can
exercise the whole UI offline). Flip the in-page switch to drive the real arm;
the prominent **STOP** button sends a controlled stop. No third-party deps (it
uses the standard-library `http.server`).

### Control from Python while the GUI shows live state

Attach the monitor to *your* robot in the same process — your script drives, the
browser displays:

```python
from ur5e_control import UR5eRobot, RobotConfig

with UR5eRobot(RobotConfig()) as robot:
    robot.serve_gui()                     # one call -> http://127.0.0.1:8080
    robot.move_l([0.1, 0.3, 0.2, 0, -3.14, 0])   # the GUI shows it live
```

A **control-mode toggle** at the top of the panel switches between **GUI
control** (jog from the browser) and **Python control** (browser locked to a
live monitor so a human can't fight the script; **STOP** always works). The lock
is enforced server-side.

### Runnable examples

```bash
python -m ur5e_control.examples.python_control      # pure Python, no GUI
python -m ur5e_control.examples.python_with_gui     # Python drives + live GUI
python -m ur5e_control.examples.move_example        # dry-run motion preview
python -m ur5e_control.examples.force_control_example   # mock force control
```
All run with no robot (dry-run / mock) by default; pass `--live` where supported
to drive real hardware.

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
  gui/             web control panel: stdlib http.server + index.html (python -m ur5e_control.gui)
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
