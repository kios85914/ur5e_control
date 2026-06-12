"""Socket transport layer for the UR5e control library.

:class:`RobotConnection` owns the raw TCP transport between this PC and the
URScript motion daemon running on the controller. Unlike most TCP clients, the
PC is the *server* here: the daemon connects **back** to ``pc_host:state_port``.
The single accepted connection is bidirectional and is used both to:

* **receive** the state stream (daemon -> PC) — newline/`+`-delimited ASCII
  frames, captured continuously by a background daemon thread; and
* **send** command tuples (PC -> daemon) — fixed-format ASCII strings.

This module deliberately knows **nothing** about motion semantics, the command
protocol, frame formats, units, or coordinate frames. It only moves bytes and
hands the latest received string back to the caller. All higher-level meaning
(encoding command tuples, parsing state frames, applying units/frames) lives in
the motion and state layers.

Threading model:

* A single background daemon thread (:meth:`_recv_loop`) owns the listening
  socket. It accepts the daemon, reads continuously into ``_latest_state``, and
  on disconnect loops back to ``accept`` again (reconnect on broken pipe).
* :meth:`send` is callable from any thread and is serialized with a send lock.
* :meth:`latest_state` is callable from any thread and reads the most recent
  complete frame under a state lock.

A ``dry_run`` flag skips all real sockets: :meth:`start`, :meth:`send` and
:meth:`close` simply log, which is useful for previewing behavior or running on
a machine with no robot/daemon present.
"""

from __future__ import annotations

import logging
import socket
import threading

from .config import RobotConfig

__all__ = ["RobotConnection"]

logger = logging.getLogger(__name__)

# Size of each TCP read. The daemon's frames are well under this; larger reads
# just coalesce multiple frames, which the receive loop handles.
_RECV_BUFSIZE = 8192

# Frame terminator emitted by the daemon. A read may contain several frames
# and/or a trailing partial frame; we always expose the last complete one.
_FRAME_TERMINATOR = "+"


class RobotConnection:
    """Bidirectional ASCII socket transport to the UR5e motion daemon.

    The PC binds and listens on ``config.pc_host:config.state_port`` and accepts
    the daemon as a client. The accepted socket carries both the inbound state
    stream and outbound command strings.

    This class is transport-only: it neither encodes commands nor parses state.
    Strings handed to :meth:`send` are ASCII-encoded verbatim, and
    :meth:`latest_state` returns the most recently received frame string as-is.

    Args:
        config: Robot configuration providing ``pc_host`` and ``state_port`` for
            the bound endpoint. (Network only; no units/frames are interpreted
            here.)
        dry_run: When ``True``, no real sockets are opened. :meth:`start`,
            :meth:`send` and :meth:`close` log instead of touching the network,
            and :meth:`latest_state` always returns ``""``.
    """

    def __init__(self, config: RobotConfig = RobotConfig(), dry_run: bool = False) -> None:
        self._config = config
        self._dry_run = dry_run

        # Listening (server) socket bound to pc_host:state_port.
        self._server_sock: socket.socket | None = None
        # The accepted daemon connection (bidirectional). None when no daemon
        # is currently connected.
        self._conn: socket.socket | None = None

        # Most recent complete state frame received from the daemon (raw string).
        self._latest_state: str = ""

        # Locks: one guards outbound sends (atomic messages), one guards access
        # to the latest-state string, one guards the accepted-connection handle.
        self._send_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._conn_lock = threading.Lock()

        # Background receive thread + its stop flag.
        self._recv_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._closed = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def start(self) -> None:
        """Bind, listen, and begin accepting the daemon in the background.

        Binds ``pc_host:state_port`` with ``SO_REUSEADDR`` so a recently closed
        port can be reused, starts listening, then launches the background
        receive thread which accepts the daemon and reads its state stream.
        Returns immediately; the daemon may connect at any later point.

        In ``dry_run`` mode this only logs and starts no thread or socket.
        """
        if self._dry_run:
            logger.info(
                "[dry_run] RobotConnection.start() — would bind %s:%d",
                self._config.pc_host,
                self._config.state_port,
            )
            return

        if self._recv_thread is not None and self._recv_thread.is_alive():
            logger.debug("RobotConnection.start() called while already started")
            return

        self._stop_event.clear()
        self._closed = False

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self._config.pc_host, self._config.state_port))
        server.listen(1)
        # Wake the accept() periodically so the stop flag is checked promptly.
        server.settimeout(0.2)
        self._server_sock = server

        self._recv_thread = threading.Thread(
            target=self._recv_loop,
            name="RobotConnection-recv",
            daemon=True,
        )
        self._recv_thread.start()
        logger.info(
            "RobotConnection listening on %s:%d",
            self._config.pc_host,
            self._config.state_port,
        )

    def close(self) -> None:
        """Shut down cleanly: stop the receive loop and release all sockets.

        Signals the background thread to stop, closes the accepted daemon
        connection and the listening socket, and joins the thread. Idempotent:
        calling it more than once (or before :meth:`start`) is safe.

        In ``dry_run`` mode this only logs.
        """
        if self._dry_run:
            logger.info("[dry_run] RobotConnection.close()")
            return

        if self._closed:
            return
        self._closed = True
        self._stop_event.set()

        # Close the accepted connection to unblock any in-progress recv().
        with self._conn_lock:
            conn = self._conn
            self._conn = None
        if conn is not None:
            self._safe_close(conn)

        # Close the listening socket to unblock accept().
        server = self._server_sock
        self._server_sock = None
        if server is not None:
            self._safe_close(server)

        # Join the receive thread (but never join ourselves).
        thread = self._recv_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)

        logger.info("RobotConnection closed")

    # ------------------------------------------------------------------
    # Send
    # ------------------------------------------------------------------
    def send(self, msg: str) -> None:
        """Send an ASCII command string to the daemon (thread-safe).

        The string is encoded as ASCII and written atomically with respect to
        other :meth:`send` callers (guarded by a lock), so concurrent senders
        never interleave bytes within a single message. No framing, encoding of
        numbers, or unit/frame interpretation is performed — ``msg`` is sent
        verbatim.

        If the daemon is not currently connected, the send is dropped with a
        warning (the next accepted daemon will receive subsequent sends).

        In ``dry_run`` mode this only logs the message.

        Args:
            msg: The exact ASCII payload to transmit.
        """
        if self._dry_run:
            logger.info("[dry_run] send: %s", msg)
            return

        data = msg.encode("ascii")
        with self._send_lock:
            with self._conn_lock:
                conn = self._conn
            if conn is None:
                logger.warning("send() dropped, no daemon connected: %s", msg)
                return
            try:
                conn.sendall(data)
            except OSError as exc:
                # Broken pipe / reset: drop this connection so the receive loop
                # re-accepts a fresh daemon.
                logger.warning("send() failed (%s); dropping connection", exc)
                self._drop_connection(conn)

    # ------------------------------------------------------------------
    # State access
    # ------------------------------------------------------------------
    def latest_state(self) -> str:
        """Return the most recently received complete state frame (thread-safe).

        Returns the raw frame string exactly as received from the daemon (no
        parsing). Returns ``""`` if no complete frame has been received yet, or
        always in ``dry_run`` mode. Reads are guarded by a lock so a concurrent
        update never yields a torn/partial string.

        Returns:
            The latest complete state frame string, or ``""`` if none.
        """
        if self._dry_run:
            return ""
        with self._state_lock:
            return self._latest_state

    def is_connected(self) -> bool:
        """Return ``True`` if a daemon is currently connected.

        Always ``False`` in ``dry_run`` mode.
        """
        if self._dry_run:
            return False
        with self._conn_lock:
            return self._conn is not None

    # ------------------------------------------------------------------
    # Background receive loop
    # ------------------------------------------------------------------
    def _recv_loop(self) -> None:
        """Accept the daemon and read its state stream until stopped.

        Runs on the background thread. Outer loop re-accepts a fresh daemon after
        any disconnect (reconnect on broken pipe); inner loop reads frames and
        publishes the latest complete one. Exits when the stop event is set.
        """
        while not self._stop_event.is_set():
            conn = self._accept_daemon()
            if conn is None:
                continue  # timed out or stopping; re-check stop flag
            logger.info("daemon connected")
            with self._conn_lock:
                self._conn = conn

            buffer = ""
            try:
                conn.settimeout(0.2)
                while not self._stop_event.is_set():
                    try:
                        chunk = conn.recv(_RECV_BUFSIZE)
                    except socket.timeout:
                        continue
                    except OSError as exc:
                        # A close()/shutdown deliberately tears the socket down;
                        # that's not a fault, so only warn for unexpected errors.
                        if self._stop_event.is_set() or self._closed:
                            break
                        logger.warning("recv() error (%s); daemon disconnected", exc)
                        break
                    if not chunk:
                        logger.info("daemon disconnected (stream closed)")
                        break
                    buffer = self._consume(buffer + chunk.decode("ascii", errors="replace"))
            finally:
                self._drop_connection(conn)

        logger.debug("receive loop exiting")

    def _accept_daemon(self) -> socket.socket | None:
        """Accept a daemon connection, returning ``None`` on timeout/stop.

        Wraps ``server.accept()``; the listening socket has a short timeout so
        the stop flag is honored promptly.
        """
        server = self._server_sock
        if server is None:
            return None
        try:
            conn, _addr = server.accept()
            return conn
        except socket.timeout:
            return None
        except OSError:
            # Listening socket closed during shutdown.
            return None

    def _consume(self, buffer: str) -> str:
        """Extract the last complete frame from ``buffer`` and publish it.

        Splits accumulated text on the ``+`` terminator. Everything up to and
        including the final ``+`` constitutes complete frame(s); the last
        complete frame (with its terminator) is published via
        :meth:`_set_latest_state`. Any trailing partial frame after the final
        ``+`` is returned to be prepended to the next read.

        Args:
            buffer: Accumulated, still-undelivered received text.

        Returns:
            The trailing partial-frame remainder to carry over.
        """
        if _FRAME_TERMINATOR not in buffer:
            return buffer

        # Split into [..complete frames.., trailing-partial]. The trailing
        # element is whatever followed the final '+'.
        *frames, remainder = buffer.split(_FRAME_TERMINATOR)
        # Publish the last non-empty complete frame, re-adding its terminator so
        # callers see the exact wire format.
        for candidate in reversed(frames):
            if candidate.strip():
                self._set_latest_state(candidate + _FRAME_TERMINATOR)
                break
        return remainder

    def _set_latest_state(self, frame: str) -> None:
        """Atomically replace the latest-state string (lock-guarded)."""
        with self._state_lock:
            self._latest_state = frame

    # ------------------------------------------------------------------
    # Internal socket helpers
    # ------------------------------------------------------------------
    def _drop_connection(self, conn: socket.socket) -> None:
        """Close ``conn`` and clear it as the active connection if it still is."""
        with self._conn_lock:
            if self._conn is conn:
                self._conn = None
        self._safe_close(conn)

    @staticmethod
    def _safe_close(sock: socket.socket) -> None:
        """Best-effort shutdown+close of a socket, swallowing errors."""
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------
    def __enter__(self) -> "RobotConnection":
        self.start()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()
