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

from ur5e_control.gui.server import RobotService, make_handler


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
    # world pose chosen so that world_to_ur (negate x,y) lands inside the
    # default workspace: ur = [0.0, 0.5, 0.2, ...] within x/y/z bounds.
    res = svc.move_l({"pose": [0.0, -0.5, 0.2, 0.0, -3.14, 0.0], "speed": 0.05})
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
    _, res = _post(server, "/api/move_l", {"pose": [0.0, -0.5, 0.2, 0.0, -3.14, 0.0]})
    assert res["ok"] is True
    _, res = _post(server, "/api/stop", {})
    assert res["ok"] is True


def test_http_unknown_route_404(server):
    req = urllib.request.Request(server + "/api/nope", data=b"{}", method="POST")
    with pytest.raises(urllib.error.HTTPError) as ei:
        urllib.request.urlopen(req, timeout=3)
    assert ei.value.code == 404
