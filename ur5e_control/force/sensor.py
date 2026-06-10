"""Force/torque sensor abstraction for the UR5e control library.

.. warning::

   **Pending hardware validation.** None of the backends in this module have
   been validated against a physically mounted Robotiq FT 300. The numeric
   read path is exercised only by mock/streamed-state tests today; on-robot
   zeroing and scaling must be tuned when the sensor is fitted.

Read-path question (spec §11)
-----------------------------
The way wrench data reaches the PC is deliberately **swappable behind the
:class:`ForceSensor` interface**:

* **Now (URCap-fed URScript stream):** the FT 300 URCap publishes wrist
  force/torque into the daemon, which folds it into the state stream. The PC
  reads it from the latest :class:`~ur5e_control.state.RobotState`
  (``state.wrench``). :class:`RobotiqFT300` implements exactly this path via an
  injected ``state_provider`` callable.
* **Later (PC-side serial/USB driver):** the FT 300 may instead be polled
  directly over RS-485/USB from the PC. That backend would be a *new*
  :class:`ForceSensor` subclass with its own ``read()``/``zero()`` — no caller
  needs to change, because everything depends only on this interface.

Units & frame
-------------
Every wrench is a 6-element list ``[fx, fy, fz, tx, ty, tz]`` with forces in
**newtons (N)** and torques in **newton-metres (Nm)**, expressed in the
**UR base frame** (consistent with :class:`~ur5e_control.state.RobotState`).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Callable

from ur5e_control.state import RobotState

__all__ = ["ForceSensor", "MockForceSensor", "RobotiqFT300"]

# A wrench is always six scalars: [fx, fy, fz, tx, ty, tz].
_WRENCH_LEN = 6


class ForceSensor(ABC):
    """Abstract 6-DOF force/torque sensor.

    Concrete backends provide a single way to obtain the current wrench and a
    way to re-tare (zero) the sensor. The interface is intentionally minimal so
    the underlying read path (URCap-fed URScript stream now, PC-side serial/USB
    driver later; see the module docstring and spec §11) can be swapped without
    touching callers.
    """

    @abstractmethod
    def read(self) -> list[float]:
        """Return the current wrench.

        Returns:
            A 6-element list ``[fx, fy, fz, tx, ty, tz]`` in newtons (forces)
            and newton-metres (torques), expressed in the UR base frame.
        """
        raise NotImplementedError

    @abstractmethod
    def zero(self) -> None:
        """Re-tare the sensor so the current load reads as zero.

        For real hardware this resets the bias offset; for replay/mock backends
        it resets the playback position. Returns nothing.
        """
        raise NotImplementedError


class MockForceSensor(ForceSensor):
    """Replay a scripted sequence of wrenches, in order, for tests/examples.

    Each call to :meth:`read` returns the next wrench in the supplied sequence.
    This is the deterministic stand-in used by the force-control unit tests
    while no FT 300 is mounted.

    Args:
        sequence: An ordered iterable of wrenches; each wrench is a 6-element
            sequence ``[fx, fy, fz, tx, ty, tz]`` (N, Nm), UR base frame.

    Raises:
        ValueError: If any wrench in ``sequence`` does not have exactly six
            elements.
    """

    def __init__(self, sequence) -> None:
        materialized: list[list[float]] = []
        for i, wrench in enumerate(sequence):
            wrench = list(wrench)
            if len(wrench) != _WRENCH_LEN:
                raise ValueError(
                    f"Wrench {i} must have {_WRENCH_LEN} elements "
                    f"[fx,fy,fz,tx,ty,tz], got {len(wrench)}"
                )
            materialized.append([float(v) for v in wrench])
        self._sequence: list[list[float]] = materialized
        self._index: int = 0

    def read(self) -> list[float]:
        """Return the next scripted wrench and advance the playback cursor.

        Returns:
            A fresh 6-element list ``[fx, fy, fz, tx, ty, tz]`` (N, Nm),
            UR base frame. A copy is returned so callers may mutate it freely.

        Raises:
            IndexError: If the scripted sequence has been exhausted.
        """
        if self._index >= len(self._sequence):
            raise IndexError("MockForceSensor sequence exhausted")
        wrench = self._sequence[self._index]
        self._index += 1
        return list(wrench)

    def zero(self) -> None:
        """Reset playback to the start of the scripted sequence."""
        self._index = 0


class RobotiqFT300(ForceSensor):
    """Robotiq FT 300 read via the daemon's streamed state (URCap-fed path).

    The wrench is pulled from the **latest** :class:`~ur5e_control.state.RobotState`
    returned by an injected ``state_provider`` callable. This keeps the read
    path swappable: today the URCap publishes the wrist wrench into the state
    stream; a future PC-side serial/USB driver would be a different
    :class:`ForceSensor` subclass entirely (see the module docstring, spec §11).

    Args:
        state_provider: A zero-argument callable returning the most recent
            :class:`~ur5e_control.state.RobotState`. Its ``wrench`` field
            (``[fx, fy, fz, tx, ty, tz]`` in N/Nm, UR base frame) is what
            :meth:`read` returns.

    Raises:
        TypeError: If ``state_provider`` is not callable.
    """

    def __init__(self, state_provider: Callable[[], RobotState]) -> None:
        if not callable(state_provider):
            raise TypeError("state_provider must be a callable returning RobotState")
        self._state_provider = state_provider

    def read(self) -> list[float]:
        """Return the wrench from the latest streamed robot state.

        Returns:
            A fresh 6-element list ``[fx, fy, fz, tx, ty, tz]`` (N, Nm),
            UR base frame, copied from the current ``RobotState.wrench`` so the
            caller cannot mutate the underlying state.
        """
        state = self._state_provider()
        return [float(v) for v in state.wrench]

    def zero(self) -> None:
        """Re-tare the FT 300.

        Pending hardware validation: with the URCap-fed streamed-force path the
        tare is performed in the URCap/daemon, so there is nothing to do on the
        PC side and this is a no-op. When a PC-side serial/USB driver replaces
        this backend, biasing will be implemented there (spec §11).
        """
        # No-op for the streamed-force backend; see docstring.
        return None
