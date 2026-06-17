"""Tests for RobotiqFT300Stream — the PC-side reader of the Robotiq port 63351.

The parser is format-tolerant (it accepts a record only if it holds exactly six
floats), and the reader runs a background thread over a real TCP socket. We test
the parser directly and exercise the socket path against a tiny local server, so
no robot/sensor is needed.
"""

import socket
import threading
import time

import pytest

from ur5e_control.force.sensor import (
    RobotiqFT300Stream,
    _extract_latest,
    _parse_record,
)


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------
def test_parse_record_parenthesised():
    assert _parse_record("( 1.0 , 2.0 , 3.0 , 0.1 , 0.2 , 0.3 )") == [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]


def test_parse_record_bare_csv():
    assert _parse_record("-1.5,2.0,3.25,0.0,-0.2,0.3") == [-1.5, 2.0, 3.25, 0.0, -0.2, 0.3]


def test_parse_record_wrong_count_is_none():
    assert _parse_record("1.0, 2.0, 3.0") is None          # too few
    assert _parse_record("1,2,3,4,5,6,7") is None           # too many (e.g. a counter)
    assert _parse_record("not numbers here") is None


def test_extract_latest_returns_last_complete_record():
    buf = "( 1,2,3,4,5,6 )( 7,8,9,10,11,12 )( 13,14"
    wrench, remainder = _extract_latest(buf)
    assert wrench == [7, 8, 9, 10, 11, 12]      # last COMPLETE record
    assert remainder == "( 13,14"                # trailing partial carried over


def test_extract_latest_no_complete_record():
    wrench, remainder = _extract_latest("( 1,2,3")
    assert wrench is None
    assert remainder == "( 1,2,3"


def test_extract_latest_newline_delimited():
    buf = "1,2,3,4,5,6\n7,8,9,10,11,12\n13,14"
    wrench, remainder = _extract_latest(buf)
    assert wrench == [7, 8, 9, 10, 11, 12]
    assert remainder == "13,14"


# ---------------------------------------------------------------------------
# Socket reader against a local fake server
# ---------------------------------------------------------------------------
class _FakeFTServer:
    """A tiny TCP server that streams a fixed Robotiq-style record."""

    def __init__(self, record: bytes):
        self._record = record
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.listen(1)
        self.port = self._sock.getsockname()[1]
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)

    def start(self):
        self._thread.start()

    def _serve(self):
        self._sock.settimeout(0.2)
        try:
            conn, _ = self._sock.accept()
        except OSError:
            return
        with conn:
            while not self._stop.is_set():
                try:
                    conn.sendall(self._record)
                except OSError:
                    break
                time.sleep(0.01)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        try:
            self._sock.close()
        except OSError:
            pass


def test_stream_reads_latest_wrench_from_socket():
    server = _FakeFTServer(b"( 1.0 , 2.0 , 3.0 , 0.1 , 0.2 , 0.3 )\n")
    server.start()
    sensor = RobotiqFT300Stream("127.0.0.1", server.port)
    sensor.start()
    try:
        assert sensor.wait_for_data(timeout=2.0), "no sample received from fake server"
        assert sensor.read() == [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]
        assert sensor.has_data()
    finally:
        sensor.close()
        server.close()


def test_stream_read_before_data_raises():
    # Point at a closed port; no data will ever arrive, read() must raise.
    sensor = RobotiqFT300Stream("127.0.0.1", 1)  # port 1: connection will fail
    sensor.start()
    try:
        with pytest.raises(ValueError):
            sensor.read()
    finally:
        sensor.close()


def test_software_tare_zeroes_current_load():
    server = _FakeFTServer(b"( 1.0 , 2.0 , 3.0 , 0.1 , 0.2 , 0.3 )\n")
    server.start()
    sensor = RobotiqFT300Stream("127.0.0.1", server.port)
    sensor.start()
    try:
        assert sensor.wait_for_data(timeout=2.0)
        offset = sensor.zero(samples=5)
        assert offset == [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]
        # read() now subtracts the offset -> ~0; read_raw() is unchanged.
        assert sensor.read() == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        assert sensor.read_raw() == [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]
        # clearing the tare restores the raw value on read().
        sensor.clear_zero()
        assert sensor.read() == [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]
    finally:
        sensor.close()
        server.close()


def test_zero_without_data_raises():
    sensor = RobotiqFT300Stream("127.0.0.1", 1)  # never connects
    sensor.start()
    try:
        with pytest.raises(ValueError):
            sensor.zero(timeout=0.3)
    finally:
        sensor.close()


class _CmdServer:
    """A one-shot TCP server that records the bytes a client sends it."""

    def __init__(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.listen(1)
        self.port = self._sock.getsockname()[1]
        self.received = b""
        self._thread = threading.Thread(target=self._serve, daemon=True)

    def start(self):
        self._thread.start()

    def _serve(self):
        self._sock.settimeout(2.0)
        try:
            conn, _ = self._sock.accept()
        except OSError:
            return
        with conn:
            conn.settimeout(1.0)
            try:
                self.received = conn.recv(64)
            except OSError:
                pass

    def close(self):
        self._thread.join(timeout=1.0)
        try:
            self._sock.close()
        except OSError:
            pass


def test_zero_via_urcap_sends_set_zro():
    server = _CmdServer()
    server.start()
    sensor = RobotiqFT300Stream("127.0.0.1", 63351)  # data port unused here
    ok = sensor.zero_via_urcap(command_port=server.port, timeout=2.0)
    server.close()
    assert ok is True
    assert server.received == b"SET ZRO"


def test_zero_via_urcap_returns_false_on_no_server():
    sensor = RobotiqFT300Stream("127.0.0.1", 63351)
    # port 1 will refuse the connection -> graceful False, no raise.
    assert sensor.zero_via_urcap(command_port=1, timeout=1.0) is False
