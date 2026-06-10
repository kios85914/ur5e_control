# `ur5e_control.force.sensor`

Force/torque sensor abstraction.

> **Pending hardware validation.** None of the backends in this module have been
> validated against a physically mounted Robotiq FT 300. The numeric read path is
> exercised only by mock/streamed-state tests today; on-robot zeroing and scaling
> must be tuned when the sensor is fitted.

## Read-path design (spec §11)

The way wrench data reaches the PC is deliberately **swappable behind the
`ForceSensor` interface**:

* **Now (URCap-fed URScript stream):** the FT 300 URCap publishes wrist
  force/torque into the daemon, which folds it into the state stream. The PC
  reads it from the latest `RobotState.wrench`. `RobotiqFT300` implements this
  path via an injected `state_provider` callable.
* **Later (PC-side serial/USB driver):** the FT 300 may instead be polled
  directly over RS-485/USB. That backend would be a *new* `ForceSensor` subclass
  with its own `read()`/`zero()` — no caller changes.

## Units & frame

Every wrench is a 6-element list `[fx, fy, fz, tx, ty, tz]` with forces in
**newtons (N)** and torques in **newton-metres (Nm)**, expressed in the **UR base
frame** (consistent with `RobotState`).

---

## `class ForceSensor(ABC)`

Abstract 6-DOF force/torque sensor. Concrete backends provide a way to obtain the
current wrench and a way to re-tare (zero). The interface is intentionally
minimal so the underlying read path can be swapped without touching callers.

### `read(self) -> list[float]`  *(abstract)*

* **Returns.** A 6-element list `[fx, fy, fz, tx, ty, tz]` in newtons (forces)
  and newton-metres (torques), expressed in the UR base frame.

### `zero(self) -> None`  *(abstract)*

Re-tare the sensor so the current load reads as zero. For real hardware this
resets the bias offset; for replay/mock backends it resets the playback position.
Returns nothing.

---

## `class MockForceSensor(ForceSensor)`

Replay a scripted sequence of wrenches, in order, for tests/examples.

### `__init__(self, sequence) -> None`

* **Parameters.** `sequence` — an ordered iterable of wrenches; each wrench is a
  6-element sequence `[fx, fy, fz, tx, ty, tz]` (N, Nm), UR base frame.
* **Exceptions.** `ValueError` if any wrench does not have exactly six elements.

### `read(self) -> list[float]`

Return the next scripted wrench and advance the playback cursor. Returns a fresh
6-element list (a copy, so callers may mutate it freely).

* **Exceptions.** `IndexError` if the scripted sequence has been exhausted.

### `zero(self) -> None`

Reset playback to the start of the scripted sequence.

---

## `class RobotiqFT300(ForceSensor)`

Robotiq FT 300 read via the daemon's streamed state (URCap-fed path). The wrench
is pulled from the **latest** `RobotState` returned by an injected
`state_provider` callable.

### `__init__(self, state_provider: Callable[[], RobotState]) -> None`

* **Parameters.** `state_provider` — a zero-argument callable returning the most
  recent `RobotState`. Its `wrench` field (`[fx, fy, fz, tx, ty, tz]` in N/Nm, UR
  base frame) is what `read` returns.
* **Exceptions.** `TypeError` if `state_provider` is not callable.

### `read(self) -> list[float]`

Return the wrench from the latest streamed robot state — a fresh 6-element list
(copied from `RobotState.wrench`, so the caller cannot mutate the underlying
state).

### `zero(self) -> None`

Re-tare the FT 300. **No-op** in the URCap-fed streamed-force path (the tare is
performed in the URCap/daemon). When a PC-side serial/USB driver replaces this
backend, biasing will be implemented there (spec §11).

### Usage example

```python
from ur5e_control.force.sensor import MockForceSensor, RobotiqFT300
from ur5e_control.robot import UR5eRobot

# Deterministic mock for tests/examples:
sensor = MockForceSensor([
    [0.0, 0.0, 0.0, 0, 0, 0],
    [0.0, 0.0, 5.0, 0, 0, 0],   # 5 N along +z
])
sensor.read()   # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
sensor.read()   # [0.0, 0.0, 5.0, 0.0, 0.0, 0.0]
sensor.zero()   # rewind to the start

# Real FT 300 fed from the streamed state (UR base frame, N/Nm):
with UR5eRobot() as robot:
    ft = RobotiqFT300(state_provider=robot.get_state)
    wrench = ft.read()   # robot.get_state().wrench, copied
```
