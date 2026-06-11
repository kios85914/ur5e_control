"""Standard-library HTTP server bridging the browser control panel to UR5eRobot.

The server owns a single :class:`~ur5e_control.robot.UR5eRobot` session and
exposes a tiny JSON API that the bundled ``index.html`` calls:

==========================  ======  =====================================================
endpoint                    method  body / returns
==========================  ======  =====================================================
``/``                       GET     serves ``index.html``
``/api/status``             GET     -> session status (connected / dry_run / daemon)
``/api/state``              GET     -> latest RobotState (pose/speed/joints/wrench) or null
``/api/connect``            POST    {controller_ip, pc_host, script_port, state_port, dry_run}
``/api/disconnect``         POST    -> status
``/api/move_l``             POST    {pose:[6], speed?, relative?}
``/api/move_j``             POST    {joints:[6], speed?}
``/api/stop``               POST    emergency/controlled stop
``/api/home``               POST    move to configured home pose
==========================  ======  =====================================================

Every robot call runs under a lock (the HTTP server is threaded) and is wrapped
so exceptions — including safety violations like ``WorkspaceViolation`` — come
back as ``{"ok": false, "error": ..., "type": ...}`` for the UI to display
rather than crashing the request. Moves are issued non-blocking so the request
returns immediately and the UI reflects motion through state polling.

Security: binds to ``127.0.0.1`` by default (local machine only).
"""

from __future__ import annotations

import json
import logging
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable

from ..config import RobotConfig
from ..robot import UR5eRobot

__all__ = ["RobotService", "make_handler", "run"]

logger = logging.getLogger(__name__)

_HTML_PATH = Path(__file__).resolve().parent / "index.html"


class RobotService:
    """Thread-safe holder of one :class:`UR5eRobot` session for the GUI.

    All methods return JSON-serializable dicts. Robot operations are serialized
    with a lock and never raise to the caller: failures (bad input, not
    connected, safety violations) come back as ``{"ok": False, "error": ...}``.
    """

    def __init__(self) -> None:
        self._robot: UR5eRobot | None = None
        self._dry_run: bool = True
        self._lock = threading.Lock()

    # -- helpers -------------------------------------------------------
    def _status_unlocked(self) -> dict[str, Any]:
        robot = self._robot
        return {
            "ok": True,
            "connected": robot is not None,
            "dry_run": self._dry_run,
            "daemon_connected": bool(robot.is_daemon_connected()) if robot else False,
        }

    @staticmethod
    def _fail(exc: Exception) -> dict[str, Any]:
        return {"ok": False, "error": str(exc), "type": type(exc).__name__}

    # -- session -------------------------------------------------------
    def connect(self, params: dict[str, Any]) -> dict[str, Any]:
        """(Re)connect a UR5eRobot built from the supplied network params."""
        with self._lock:
            try:
                if self._robot is not None:
                    self._robot.disconnect()
                    self._robot = None
                dry_run = bool(params.get("dry_run", True))
                config = RobotConfig(
                    controller_ip=str(params.get("controller_ip", RobotConfig().controller_ip)),
                    pc_host=str(params.get("pc_host", RobotConfig().pc_host)),
                    script_port=int(params.get("script_port", RobotConfig().script_port)),
                    state_port=int(params.get("state_port", RobotConfig().state_port)),
                )
                robot = UR5eRobot(config, dry_run=dry_run)
                robot.connect()
                self._robot = robot
                self._dry_run = dry_run
                return self._status_unlocked()
            except Exception as exc:  # surface to the UI, don't crash the request
                logger.warning("connect failed: %s", exc)
                return self._fail(exc)

    def disconnect(self) -> dict[str, Any]:
        with self._lock:
            try:
                if self._robot is not None:
                    self._robot.disconnect()
            except Exception as exc:
                return self._fail(exc)
            finally:
                self._robot = None
            return self._status_unlocked()

    def status(self) -> dict[str, Any]:
        with self._lock:
            return self._status_unlocked()

    def state(self) -> dict[str, Any]:
        """Return the latest robot state, or ``state: None`` if none yet."""
        with self._lock:
            robot = self._robot
            base = self._status_unlocked()
            if robot is None:
                base["state"] = None
                return base
            try:
                st = robot.get_state()
                base["state"] = {
                    "tcp_pose": list(st.tcp_pose),
                    "tcp_speed": list(st.tcp_speed),
                    "joints": list(st.joints),
                    "wrench": list(st.wrench),
                    "timestamp": st.timestamp,
                }
            except Exception:
                # No complete frame yet (common right after connect / in dry-run).
                base["state"] = None
            return base

    # -- motion --------------------------------------------------------
    def _with_robot(self, fn: Callable[[UR5eRobot], Any]) -> dict[str, Any]:
        with self._lock:
            if self._robot is None:
                return {"ok": False, "error": "not connected", "type": "NotConnected"}
            try:
                fn(self._robot)
                return {"ok": True}
            except Exception as exc:
                logger.warning("robot op failed: %s", exc)
                return self._fail(exc)

    def move_l(self, params: dict[str, Any]) -> dict[str, Any]:
        pose = [float(v) for v in params["pose"]]
        speed = params.get("speed")
        speed = float(speed) if speed is not None else None
        relative = bool(params.get("relative", False))
        return self._with_robot(
            lambda r: r.move_l(pose, speed=speed, blocking=False, relative=relative)
        )

    def move_j(self, params: dict[str, Any]) -> dict[str, Any]:
        joints = [float(v) for v in params["joints"]]
        speed = params.get("speed")
        speed = float(speed) if speed is not None else None
        return self._with_robot(lambda r: r.move_j(joints, speed=speed, blocking=False))

    def stop(self) -> dict[str, Any]:
        return self._with_robot(lambda r: r.stop())

    def home(self) -> dict[str, Any]:
        return self._with_robot(lambda r: r.home(blocking=False))


# Route table: (method, path) -> (service-method-name, needs_body)
_GET_ROUTES = {"/api/status": "status", "/api/state": "state"}
_POST_ROUTES = {
    "/api/connect": ("connect", True),
    "/api/disconnect": ("disconnect", False),
    "/api/move_l": ("move_l", True),
    "/api/move_j": ("move_j", True),
    "/api/stop": ("stop", False),
    "/api/home": ("home", False),
}


def make_handler(service: RobotService, html_path: Path = _HTML_PATH):
    """Build a request handler class bound to ``service`` and the HTML file."""

    class Handler(BaseHTTPRequestHandler):
        # quiet default logging; route through our logger instead
        def log_message(self, fmt: str, *args: Any) -> None:  # noqa: A003
            logger.debug("%s - %s", self.address_string(), fmt % args)

        def _send_json(self, payload: dict[str, Any], code: int = 200) -> None:
            body = json.dumps(payload).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802
            if self.path in ("/", "/index.html"):
                try:
                    body = html_path.read_bytes()
                except OSError:
                    self._send_json({"ok": False, "error": "index.html missing"}, 500)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path in _GET_ROUTES:
                self._send_json(getattr(service, _GET_ROUTES[self.path])())
                return
            self._send_json({"ok": False, "error": "not found"}, 404)

        def do_POST(self) -> None:  # noqa: N802
            route = _POST_ROUTES.get(self.path)
            if route is None:
                self._send_json({"ok": False, "error": "not found"}, 404)
                return
            method_name, needs_body = route
            try:
                params: dict[str, Any] = {}
                if needs_body:
                    length = int(self.headers.get("Content-Length", 0))
                    raw = self.rfile.read(length) if length else b"{}"
                    params = json.loads(raw or b"{}")
                fn = getattr(service, method_name)
                result = fn(params) if needs_body else fn()
                self._send_json(result)
            except Exception as exc:  # malformed request body, etc.
                self._send_json({"ok": False, "error": str(exc), "type": type(exc).__name__}, 400)

    return Handler


def run(host: str = "127.0.0.1", port: int = 8080) -> None:
    """Start the control-panel server (blocking) on ``host:port``."""
    service = RobotService()
    httpd = ThreadingHTTPServer((host, port), make_handler(service))
    url = f"http://{host}:{port}"
    print(f"ur5e_control control panel -> {url}  (dry-run by default; Ctrl-C to stop)")
    logger.info("serving on %s", url)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down…")
    finally:
        httpd.shutdown()
        service.disconnect()
