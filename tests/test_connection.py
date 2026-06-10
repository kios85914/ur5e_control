"""Tests for ur5e_control.connection — RobotConnection socket transport.

The PC side binds ``pc_host:state_port``, listens, and accepts the daemon as a
client. The single accepted connection is used both to receive the state stream
(daemon -> PC) and to send command tuples (PC -> daemon). All values exchanged
on the wire are ASCII strings; this layer knows nothing about motion semantics.

These tests run entirely on the loopback interface (``127.0.0.1``) using an
ephemeral port via a test-only :class:`RobotConfig`, so no real robot or daemon
is required. A small in-process "fake daemon" connects back to the bound socket,
the way the real URScript daemon does.
"""

import socket
import threading
import time

import pytest

from ur5e_control.config import RobotConfig
from ur5e_control.connection import RobotConnection


# ---------------------------------------------------------------------------
# Helpers / fixtures
# ---------------------------------------------------------------------------

def _free_port() -> int:
    """Return an OS-assigned free TCP port on the loopback interface."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture
def loopback_config() -> RobotConfig:
    """A RobotConfig whose state endpoint is a loopback ephemeral port."""
    return RobotConfig(pc_host="127.0.0.1", state_port=_free_port())


def _connect_fake_daemon(host: str, port: int, timeout: float = 2.0) -> socket.socket:
    """Open a client socket to the bound RobotConnection, retrying briefly.

    Mimics the real daemon connecting back to ``pc_host:state_port``.
    """
    deadline = time.time() + timeout
    last_err = None
    while time.time() < deadline:
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect((host, port))
            return client
        except OSError as exc:  # server not yet accepting
            last_err = exc
            try:
                client.close()
            except OSError:
                pass
            time.sleep(0.01)
    raise AssertionError(f"fake daemon could not connect: {last_err}")


def _wait_until(predicate, timeout: float = 2.0, interval: float = 0.005) -> bool:
    """Poll ``predicate`` until truthy or timeout. Returns the final truthiness."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return bool(predicate())


# ---------------------------------------------------------------------------
# start() / accept
# ---------------------------------------------------------------------------

def test_start_binds_and_accepts_daemon(loopback_config):
    conn = RobotConnection(loopback_config)
    try:
        conn.start()
        client = _connect_fake_daemon(loopback_config.pc_host, loopback_config.state_port)
        try:
            assert _wait_until(conn.is_connected), "connection never reported connected"
        finally:
            client.close()
    finally:
        conn.close()


# ---------------------------------------------------------------------------
# send()
# ---------------------------------------------------------------------------

def test_send_writes_ascii_bytes_to_daemon(loopback_config):
    conn = RobotConnection(loopback_config)
    try:
        conn.start()
        client = _connect_fake_daemon(loopback_config.pc_host, loopback_config.state_port)
        client.settimeout(2.0)
        try:
            assert _wait_until(conn.is_connected)
            msg = "(0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.1, 0.1, 2.0)"
            conn.send(msg)

            received = b""
            # Read until we have at least the whole message.
            while len(received) < len(msg):
                chunk = client.recv(4096)
                if not chunk:
                    break
                received += chunk
            assert received.decode("ascii") == msg
        finally:
            client.close()
    finally:
        conn.close()


def test_send_is_thread_safe_under_concurrent_callers(loopback_config):
    """Concurrent send() calls must not interleave bytes within a message."""
    conn = RobotConnection(loopback_config)
    try:
        conn.start()
        client = _connect_fake_daemon(loopback_config.pc_host, loopback_config.state_port)
        client.settimeout(3.0)
        try:
            assert _wait_until(conn.is_connected)

            # Each message is the same fixed length and newline-terminated so we
            # can verify atomicity by splitting on the delimiter.
            n = 50
            msgs = [f"(MSG{ i:03d})\n" for i in range(n)]

            def worker(m: str) -> None:
                conn.send(m)

            threads = [threading.Thread(target=worker, args=(m,)) for m in msgs]
            for t in threads:
                t.start()
            for t in threads:
                t.join()

            expected_bytes = sum(len(m) for m in msgs)
            received = b""
            while len(received) < expected_bytes:
                chunk = client.recv(4096)
                if not chunk:
                    break
                received += chunk

            text = received.decode("ascii")
            lines = [ln for ln in text.split("\n") if ln]
            # Every message must appear intact and exactly once (no interleave).
            assert sorted(lines) == sorted(m.strip() for m in msgs)
        finally:
            client.close()
    finally:
        conn.close()


# ---------------------------------------------------------------------------
# latest_state() — receive loop
# ---------------------------------------------------------------------------

def test_receive_loop_captures_latest_state(loopback_config):
    conn = RobotConnection(loopback_config)
    try:
        conn.start()
        client = _connect_fake_daemon(loopback_config.pc_host, loopback_config.state_port)
        try:
            assert _wait_until(conn.is_connected)

            frame1 = (
                "p[1.0,1.0,1.0,1.0,1.0,1.0]_p[0,0,0,0,0,0]_"
                "[0,0,0,0,0,0]_p[0,0,0,0,0,0]+"
            )
            client.sendall(frame1.encode("ascii"))
            assert _wait_until(lambda: conn.latest_state() == frame1), (
                f"expected first frame, got {conn.latest_state()!r}"
            )

            frame2 = (
                "p[9.0,8.0,7.0,6.0,5.0,4.0]_p[0,0,0,0,0,0]_"
                "[0,0,0,0,0,0]_p[3,2,1,0,0,0]+"
            )
            client.sendall(frame2.encode("ascii"))
            assert _wait_until(lambda: conn.latest_state() == frame2), (
                f"expected latest frame, got {conn.latest_state()!r}"
            )
        finally:
            client.close()
    finally:
        conn.close()


def test_latest_state_is_lock_guarded_under_concurrent_reads(loopback_config):
    """Concurrent readers must always observe a complete frame, never a tear."""
    conn = RobotConnection(loopback_config)
    try:
        conn.start()
        client = _connect_fake_daemon(loopback_config.pc_host, loopback_config.state_port)
        try:
            assert _wait_until(conn.is_connected)

            valid_frames = {
                f"p[{v},{v},{v},{v},{v},{v}]_p[0,0,0,0,0,0]_"
                f"[0,0,0,0,0,0]_p[0,0,0,0,0,0]+"
                for v in (1.0, 2.0, 3.0, 4.0, 5.0)
            }

            stop = threading.Event()
            seen = []
            errors = []

            def reader() -> None:
                while not stop.is_set():
                    s = conn.latest_state()
                    if s:
                        seen.append(s)

            readers = [threading.Thread(target=reader) for _ in range(4)]
            for r in readers:
                r.start()

            try:
                for frame in valid_frames:
                    client.sendall(frame.encode("ascii"))
                    time.sleep(0.01)
                time.sleep(0.05)
            finally:
                stop.set()
                for r in readers:
                    r.join()

            assert not errors, errors
            # Anything a reader observed must be a complete, valid frame.
            for s in seen:
                assert s in valid_frames, f"observed torn/partial frame: {s!r}"
        finally:
            client.close()
    finally:
        conn.close()


def test_latest_state_empty_before_any_frame(loopback_config):
    conn = RobotConnection(loopback_config)
    try:
        conn.start()
        assert conn.latest_state() == ""
    finally:
        conn.close()


# ---------------------------------------------------------------------------
# dry_run mode
# ---------------------------------------------------------------------------

def test_dry_run_opens_no_socket_and_send_is_noop(loopback_config):
    """In dry_run, start()/send()/close() must not touch real sockets."""
    conn = RobotConnection(loopback_config, dry_run=True)
    conn.start()

    # No server should be bound on the configured port in dry_run mode, so a
    # probe connection must fail (connection refused).
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    probe.settimeout(0.3)
    with pytest.raises(OSError):
        probe.connect((loopback_config.pc_host, loopback_config.state_port))
    probe.close()

    # send() must not raise and must be a logged no-op.
    conn.send("(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)")
    assert conn.latest_state() == ""
    conn.close()


def test_dry_run_logs_sends(loopback_config, caplog):
    import logging

    conn = RobotConnection(loopback_config, dry_run=True)
    conn.start()
    with caplog.at_level(logging.INFO):
        conn.send("(1, 2, 3)")
    assert any("1, 2, 3" in rec.getMessage() for rec in caplog.records)
    conn.close()


# ---------------------------------------------------------------------------
# close()
# ---------------------------------------------------------------------------

def test_close_is_clean_and_idempotent(loopback_config):
    conn = RobotConnection(loopback_config)
    conn.start()
    client = _connect_fake_daemon(loopback_config.pc_host, loopback_config.state_port)
    assert _wait_until(conn.is_connected)
    client.close()

    conn.close()
    assert not conn.is_connected()
    # The receive thread must have stopped.
    assert _wait_until(lambda: not conn._recv_thread.is_alive())  # type: ignore[attr-defined]
    # Calling close() again must be safe.
    conn.close()


def test_close_releases_port_for_rebind(loopback_config):
    """After close(), the port must be free so a new connection can rebind it."""
    conn1 = RobotConnection(loopback_config)
    conn1.start()
    conn1.close()

    conn2 = RobotConnection(loopback_config)
    try:
        conn2.start()  # must not raise "address already in use"
        client = _connect_fake_daemon(loopback_config.pc_host, loopback_config.state_port)
        assert _wait_until(conn2.is_connected)
        client.close()
    finally:
        conn2.close()


# ---------------------------------------------------------------------------
# reconnect on broken pipe
# ---------------------------------------------------------------------------

def test_reconnect_after_daemon_drops(loopback_config):
    """If the daemon disconnects, the connection accepts a fresh client."""
    conn = RobotConnection(loopback_config)
    try:
        conn.start()
        client1 = _connect_fake_daemon(loopback_config.pc_host, loopback_config.state_port)
        assert _wait_until(conn.is_connected)

        # Daemon drops.
        client1.close()
        assert _wait_until(lambda: not conn.is_connected(), timeout=3.0)

        # Daemon reconnects; the connection must accept it again.
        client2 = _connect_fake_daemon(loopback_config.pc_host, loopback_config.state_port)
        try:
            assert _wait_until(conn.is_connected, timeout=3.0), "did not reaccept daemon"
            frame = (
                "p[5.0,5.0,5.0,5.0,5.0,5.0]_p[0,0,0,0,0,0]_"
                "[0,0,0,0,0,0]_p[0,0,0,0,0,0]+"
            )
            client2.sendall(frame.encode("ascii"))
            assert _wait_until(lambda: conn.latest_state() == frame)
        finally:
            client2.close()
    finally:
        conn.close()
