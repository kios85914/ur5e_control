"""Tests for the GUI bridge (ur5e_control.gui.server).

All run in dry-run with no robot and no real controller. The HTTP layer is
exercised against a live ThreadingHTTPServer on an ephemeral localhost port via
urllib (standard library only).
"""

from __future__ import annotations

import json
import threading
import urllib.request
from http.server import ThreadingHTTPServer

import pytest

from ur5e_control import RobotConfig, UR5eRobot
from ur5e_control.gui.server import RobotService, make_handler, serve_in_background

# a valid pose inside the default UR-frame workspace (y within 0.25..0.80)
_OK_POSE = [-0.06, 0.30, 0.20, 0.0, -3.14, 0.0]


def _attached_service():
    """A RobotService attached to an externally-owned dry-run robot."""
    robot = UR5eRobot(RobotConfig(), dry_run=True)
    robot.connect()
    return RobotService(robot=robot)


# --------------------------------------------------------------------------
# RobotService logic (no HTTP)
# --------------------------------------------------------------------------
def test_connect_dry_run_reports_status():
    svc = RobotService()
    res = svc.connect({"dry_run": True})
    assert res["ok"] is True
    assert res["connected"] is True
    assert res["dry_run"] is True
    assert res["daemon_connected"] is False  # dry-run never has a real daemon


def test_state_is_none_when_no_frame_yet():
    svc = RobotService()
    svc.connect({"dry_run": True})
    res = svc.state()
    assert res["ok"] is True
    assert res["connected"] is True
    assert res["state"] is None  # dry-run streams nothing


def test_move_l_not_connected_fails_cleanly():
    svc = RobotService()
    res = svc.move_l({"pose": [0, 0, 0, 0, 0, 0]})
    assert res["ok"] is False
    assert res["type"] == "NotConnected"


def test_move_l_valid_pose_ok_in_dry_run():
    svc = RobotService()
    svc.connect({"dry_run": True})
    # a valid UR-frame pose (y within the 0.25..0.80 workspace bound).
    res = svc.move_l({"pose": [-0.06, 0.30, 0.20, 0.0, -3.14, 0.0], "speed": 0.05})
    assert res["ok"] is True


def test_move_l_out_of_workspace_surfaces_safety_violation():
    svc = RobotService()
    svc.connect({"dry_run": True})
    res = svc.move_l({"pose": [0.0, -0.5, 5.0, 0.0, 0.0, 0.0]})  # z=5 m is out of bounds
    assert res["ok"] is False
    assert res["type"] == "WorkspaceViolation"


def test_move_j_valid_ok_and_stop_home_ok():
    svc = RobotService()
    svc.connect({"dry_run": True})
    assert svc.move_j({"joints": [0, 0, 0, 0, 0, 0], "speed": 0.1})["ok"] is True
    assert svc.stop()["ok"] is True
    assert svc.home()["ok"] is True


def test_disconnect_clears_session():
    svc = RobotService()
    svc.connect({"dry_run": True})
    res = svc.disconnect()
    assert res["ok"] is True
    assert res["connected"] is False


def test_standalone_defaults_to_gui_mode_and_can_move():
    svc = RobotService()
    svc.connect({"dry_run": True})
    assert svc.status()["mode"] == "gui"
    assert svc.status()["attached"] is False
    assert svc.move_l({"pose": _OK_POSE})["ok"] is True


# --------------------------------------------------------------------------
# Attached mode + server-enforced control modes
# --------------------------------------------------------------------------
def test_attached_defaults_to_python_mode():
    svc = _attached_service()
    s = svc.status()
    assert s["attached"] is True
    assert s["mode"] == "python"
    assert s["connected"] is True


def test_python_mode_locks_gui_moves_but_always_allows_stop():
    svc = _attached_service()
    assert svc.move_l({"pose": _OK_POSE})["type"] == "Locked"
    assert svc.move_j({"joints": [0, 0, 0, 0, 0, 0]})["type"] == "Locked"
    assert svc.home()["type"] == "Locked"
    assert svc.stop()["ok"] is True  # emergency stop is never gated


def test_switch_to_gui_mode_unlocks_moves():
    svc = _attached_service()
    assert svc.set_mode({"mode": "gui"})["mode"] == "gui"
    assert svc.move_l({"pose": _OK_POSE})["ok"] is True


def test_attached_connect_rejected_disconnect_keeps_robot():
    svc = _attached_service()
    assert svc.connect({"dry_run": True})["type"] == "Attached"
    assert svc.disconnect()["connected"] is True  # not torn down (Python owns it)


def test_set_mode_validates():
    svc = _attached_service()
    assert svc.set_mode({"mode": "bogus"})["ok"] is False


def test_standalone_python_mode_locks_moves():
    svc = RobotService()
    svc.connect({"dry_run": True})
    svc.set_mode({"mode": "python"})
    assert svc.move_l({"pose": _OK_POSE})["type"] == "Locked"
    assert svc.stop()["ok"] is True


def test_serve_in_background_attaches_and_serves():
    robot = UR5eRobot(RobotConfig(), dry_run=True)
    robot.connect()
    httpd = serve_in_background(robot, port=0)  # ephemeral port
    try:
        host, port = httpd.server_address
        with urllib.request.urlopen(f"http://{host}:{port}/api/status", timeout=3) as r:
            s = json.loads(r.read())
        assert s["attached"] is True
        assert s["mode"] == "python"
    finally:
        httpd.shutdown()


def test_robot_serve_gui_one_call_attaches():
    """UR5eRobot.serve_gui() opens the GUI attached to itself, no extra wiring."""
    robot = UR5eRobot(RobotConfig(), dry_run=True)
    robot.connect()
    httpd = robot.serve_gui(port=0)
    try:
        host, port = httpd.server_address
        with urllib.request.urlopen(f"http://{host}:{port}/api/status", timeout=3) as r:
            s = json.loads(r.read())
        assert s["attached"] is True
        assert s["mode"] == "python"
    finally:
        httpd.shutdown()


# --------------------------------------------------------------------------
# HTTP layer
# --------------------------------------------------------------------------
@pytest.fixture()
def server():
    svc = RobotService()
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(svc))
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    host, port = httpd.server_address
    yield f"http://{host}:{port}"
    httpd.shutdown()
    thread.join(timeout=2)


def _get(base, path):
    with urllib.request.urlopen(base + path, timeout=3) as r:
        return r.status, r.read()


def _post(base, path, body):
    req = urllib.request.Request(
        base + path,
        data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=3) as r:
        return r.status, json.loads(r.read())


def test_http_serves_index(server):
    status, body = _get(server, "/")
    assert status == 200
    assert b"control panel" in body.lower()


def test_http_status_endpoint(server):
    status, body = _get(server, "/api/status")
    assert status == 200
    assert json.loads(body)["ok"] is True


def test_http_connect_then_move_flow(server):
    _, res = _post(server, "/api/connect", {"dry_run": True})
    assert res["ok"] is True and res["dry_run"] is True
    _, res = _post(server, "/api/move_l", {"pose": [-0.06, 0.30, 0.20, 0.0, -3.14, 0.0]})
    assert res["ok"] is True
    _, res = _post(server, "/api/stop", {})
    assert res["ok"] is True


def test_http_unknown_route_404(server):
    req = urllib.request.Request(server + "/api/nope", data=b"{}", method="POST")
    with pytest.raises(urllib.error.HTTPError) as ei:
        urllib.request.urlopen(req, timeout=3)
    assert ei.value.code == 404
