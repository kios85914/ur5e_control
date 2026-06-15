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

import logging
import re
import socket
import threading
from abc import ABC, abstractmethod
from typing import Callable, Optional

from ur5e_control.state import RobotState

__all__ = ["ForceSensor", "MockForceSensor", "RobotiqFT300", "RobotiqFT300Stream"]

logger = logging.getLogger(__name__)

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


# Matches a single signed decimal/scientific float (used to pull the six wrench
# values out of a Robotiq 63351 record regardless of its exact punctuation).
_FLOAT_RE = re.compile(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?")

# Size of each TCP read from the Robotiq stream.
_RECV_BUFSIZE = 4096


def _parse_record(record: str) -> Optional[list[float]]:
    """Parse one Robotiq stream record into a wrench, or ``None`` if it isn't one.

    A record is the text of a single sample (e.g. ``"( 1.2 , 3.4 , 5.6 , 0.1 ,
    0.2 , 0.3 )"`` or a bare CSV line). We simply extract every float in the
    text and accept it **only if there are exactly six** — so a partial line, a
    line with a leading counter/timestamp, or any other shape is rejected rather
    than silently mis-parsed.

    Args:
        record: The text of one candidate record (without the delimiter).

    Returns:
        ``[fx, fy, fz, tx, ty, tz]`` if the record holds exactly six floats,
        else ``None``.
    """
    nums = _FLOAT_RE.findall(record)
    if len(nums) != _WRENCH_LEN:
        return None
    return [float(n) for n in nums]


def _extract_latest(buffer: str) -> tuple[Optional[list[float]], str]:
    """Pull the most recent complete wrench record from ``buffer``.

    Splits on the record delimiter — ``)`` for the parenthesised Robotiq format
    ``( ... )``, else newline for a bare CSV stream — and returns the last
    complete record that parses to six floats, plus the trailing partial
    remainder to carry into the next read.

    Args:
        buffer: Accumulated, not-yet-consumed text from the stream.

    Returns:
        ``(wrench_or_None, remainder)``.
    """
    if ")" in buffer:
        delim = ")"
    elif "\n" in buffer:
        delim = "\n"
    else:
        return None, buffer
    *records, remainder = buffer.split(delim)
    for record in reversed(records):
        wrench = _parse_record(record)
        if wrench is not None:
            return wrench, remainder
    return None, remainder


class RobotiqFT300Stream(ForceSensor):
    """Robotiq FT 300 / FT 300-S read directly from the URCap's TCP port 63351.

    The Robotiq Force Torque Sensor URCap runs on the UR controller, reads the
    sensor over RS-485, and re-publishes the live wrench as an ASCII stream on
    **TCP port 63351** of the controller. This backend connects to that port from
    the PC (over the existing network link to the controller — no extra cable),
    runs a background thread that keeps the latest sample, and exposes it through
    the standard :class:`ForceSensor` interface.

    Unlike :class:`RobotiqFT300` (which reads ``state.wrench`` — i.e. the UR's own
    ``get_tcp_force()``, NOT the FT 300-S), this reads the **actual FT 300-S**.
    Use it when the sensor is wired and the URCap reports it connected.

    The reader is resilient: a short socket timeout lets it honour :meth:`close`
    promptly, and a dropped connection is retried automatically.

    Units & frame: the stream's six values are forwarded verbatim as
    ``[fx, fy, fz, tx, ty, tz]`` (N, Nm). Note Robotiq reports them in the
    **sensor frame** (already bias-corrected by the URCap's zeroing), which is not
    necessarily the UR base frame — handle any mounting rotation downstream.

    Args:
        host: The UR controller's IP/hostname (the port-63351 server runs there).
        port: The Robotiq stream port. Defaults to ``63351``.
        connect_timeout: Seconds to wait when (re)establishing the TCP connection.
        reconnect_delay: Seconds to wait before retrying after a drop/failure.
    """

    def __init__(
        self,
        host: str,
        port: int = 63351,
        connect_timeout: float = 5.0,
        reconnect_delay: float = 0.5,
    ) -> None:
        self._host = host
        self._port = int(port)
        self._connect_timeout = connect_timeout
        self._reconnect_delay = reconnect_delay

        self._latest: Optional[list[float]] = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._closed = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def start(self) -> None:
        """Begin reading the stream on a background daemon thread (idempotent)."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._closed = False
        self._thread = threading.Thread(
            target=self._recv_loop, name="RobotiqFT300Stream-recv", daemon=True
        )
        self._thread.start()
        logger.info("RobotiqFT300Stream reading %s:%d", self._host, self._port)

    def close(self) -> None:
        """Stop the background reader and release the socket (idempotent)."""
        if self._closed:
            return
        self._closed = True
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        logger.info("RobotiqFT300Stream closed")

    def __enter__(self) -> "RobotiqFT300Stream":
        self.start()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    # ------------------------------------------------------------------
    # ForceSensor interface
    # ------------------------------------------------------------------
    def has_data(self) -> bool:
        """Return ``True`` once at least one sample has been received."""
        with self._lock:
            return self._latest is not None

    def wait_for_data(self, timeout: float = 5.0, poll: float = 0.05) -> bool:
        """Block until the first sample arrives, or ``timeout``. Returns success."""
        import time

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.has_data():
                return True
            time.sleep(poll)
        return self.has_data()

    def read(self) -> list[float]:
        """Return the most recent FT 300-S wrench.

        Returns:
            A fresh ``[fx, fy, fz, tx, ty, tz]`` (N, Nm) copy of the latest sample.

        Raises:
            ValueError: If no sample has been received yet (call :meth:`start`
                first and/or :meth:`wait_for_data`).
        """
        with self._lock:
            latest = self._latest
        if latest is None:
            raise ValueError(
                f"no FT 300-S sample received yet from {self._host}:{self._port}"
            )
        return list(latest)

    def zero(self) -> None:
        """Re-tare the FT 300-S.

        Zeroing is performed by the Robotiq URCap on the controller (it shifts the
        bias the 63351 stream is reported against), so there is nothing to do from
        the PC here — run the zero in PolyScope / via ``rq_zero_sensor()``. No-op.
        """
        return None

    # ------------------------------------------------------------------
    # Background reader
    # ------------------------------------------------------------------
    def _recv_loop(self) -> None:
        """Connect, read, and publish the latest wrench until stopped."""
        import time

        while not self._stop_event.is_set():
            try:
                sock = socket.create_connection(
                    (self._host, self._port), timeout=self._connect_timeout
                )
            except OSError as exc:
                if self._stop_event.is_set():
                    break
                logger.warning("FT300 connect to %s:%d failed (%s); retrying",
                               self._host, self._port, exc)
                time.sleep(self._reconnect_delay)
                continue

            buffer = ""
            try:
                sock.settimeout(0.2)
                while not self._stop_event.is_set():
                    try:
                        chunk = sock.recv(_RECV_BUFSIZE)
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    if not chunk:
                        break
                    buffer += chunk.decode("ascii", errors="replace")
                    wrench, buffer = _extract_latest(buffer)
                    if wrench is not None:
                        with self._lock:
                            self._latest = wrench
                    # Keep the carried-over remainder bounded if no record parses.
                    if len(buffer) > _RECV_BUFSIZE:
                        buffer = buffer[-_RECV_BUFSIZE:]
            finally:
                try:
                    sock.close()
                except OSError:
                    pass
            if not self._stop_event.is_set():
                time.sleep(self._reconnect_delay)

        logger.debug("FT300 receive loop exiting")
