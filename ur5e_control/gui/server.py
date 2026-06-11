"""Standard-library HTTP server bridging the browser control panel to UR5eRobot.

The server owns (or *attaches to*) a single :class:`~ur5e_control.robot.UR5eRobot`
and exposes a tiny JSON API that the bundled ``index.html`` calls:

==========================  ======  =====================================================
endpoint                    method  body / returns
==========================  ======  =====================================================
``/``                       GET     serves ``index.html``
``/api/status``             GET     -> session status (connected / dry_run / daemon / mode / attached)
``/api/state``              GET     -> latest RobotState (pose/speed/joints/wrench) or null
``/api/connect``            POST    {controller_ip, pc_host, script_port, state_port, dry_run}
``/api/disconnect``         POST    -> status
``/api/mode``               POST    {mode: "gui"|"python"}  switch control mode
``/api/move_l``             POST    {pose:[6], speed?, relative?}   (gated by mode)
``/api/move_j``             POST    {joints:[6], speed?}            (gated by mode)
``/api/stop``               POST    controlled stop                (ALWAYS allowed)
``/api/home``               POST    move to configured home pose   (gated by mode)
==========================  ======  =====================================================

Two ways to use it:

* **Standalone** (``python -m ur5e_control.gui``): the server creates and owns the
  robot via ``/connect``. Default control mode is ``"gui"`` — you drive from the
  browser.
* **Attached** (:func:`serve_in_background` from your own script): the server
  shares the ``UR5eRobot`` your Python code already created and connected. Default
  control mode is ``"python"`` — your script drives, the browser is a live
  monitor. The Connection panel is hidden (Python owns the lifecycle).

Control modes (server-enforced):

* ``"gui"``    — browser move/jog/home commands are honored.
* ``"python"`` — browser move/jog/home are **rejected** (``Locked``) so a human
  can't fight the controlling script. ``stop`` is **always** honored (emergency).

Every robot call runs under a lock (the HTTP server is threaded) and is wrapped so
exceptions — including safety violations like ``WorkspaceViolation`` — come back as
``{"ok": false, ...}`` for the UI to display. Moves are issued non-blocking.

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

__all__ = ["RobotService", "make_handler", "run", "serve_in_background"]

logger = logging.getLogger(__name__)

_HTML_PATH = Path(__file__).resolve().parent / "index.html"


class RobotService:
    """Thread-safe holder of one :class:`UR5eRobot` session for the GUI.

    Args:
        robot: An externally-created, already-(or about-to-be-)connected robot to
            *attach* to. When given, the service is in "attached" mode: it does
            not create or tear down the robot (your Python code owns its
            lifecycle), and the default control mode is ``"python"``. When
            ``None`` (standalone), the service creates the robot on ``connect()``
            and defaults to ``"gui"`` control mode.

    All methods return JSON-serializable dicts and never raise to the caller.
    """

    def __init__(self, robot: UR5eRobot | None = None) -> None:
        self._robot = robot
        self._attached = robot is not None
        self._dry_run = bool(robot.dry_run) if robot is not None else True
        self._mode = "python" if self._attached else "gui"
        self._lock = threading.Lock()

    # -- helpers -------------------------------------------------------
    def _status_unlocked(self) -> dict[str, Any]:
        robot = self._robot
        return {
            "ok": True,
            "connected": robot is not None,
            "dry_run": self._dry_run,
            "daemon_connected": bool(robot.is_daemon_connected()) if robot else False,
            "mode": self._mode,        # "gui" | "python"
            "attached": self._attached,  # True => Python owns the connection
        }

    @staticmethod
    def _fail(exc: Exception) -> dict[str, Any]:
        return {"ok": False, "error": str(exc), "type": type(exc).__name__}

    def _op(self, fn: Callable[[UR5eRobot], Any], gated: bool) -> dict[str, Any]:
        """Run a robot op under lock. ``gated`` ops are blocked in python mode."""
        with self._lock:
            if gated and self._mode == "python":
                return {
                    "ok": False,
                    "error": "GUI control is locked (Python control mode)",
                    "type": "Locked",
                }
            if self._robot is None:
                return {"ok": False, "error": "not connected", "type": "NotConnected"}
            try:
                fn(self._robot)
                return {"ok": True}
            except Exception as exc:
                logger.warning("robot op failed: %s", exc)
                return self._fail(exc)

    # -- session -------------------------------------------------------
    def connect(self, params: dict[str, Any]) -> dict[str, Any]:
        """(Re)connect a UR5eRobot built from the params (standalone mode only)."""
        with self._lock:
            if self._attached:
                return {
                    "ok": False,
                    "error": "robot is owned by a Python session; connect from Python",
                    "type": "Attached",
                    **{k: v for k, v in self._status_unlocked().items() if k != "ok"},
                }
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
            except Exception as exc:
                logger.warning("connect failed: %s", exc)
                return self._fail(exc)

    def disconnect(self) -> dict[str, Any]:
        with self._lock:
            if self._attached:
                # Don't tear down a robot owned by a Python session.
                return self._status_unlocked()
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

    def set_mode(self, params: dict[str, Any]) -> dict[str, Any]:
        mode = str(params.get("mode", "")).lower()
        if mode not in ("gui", "python"):
            return {"ok": False, "error": "mode must be 'gui' or 'python'", "type": "BadMode"}
        with self._lock:
            self._mode = mode
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
                base["state"] = None  # no complete frame yet / dry-run
            return base

    # -- motion (move/jog/home are gated; stop is always allowed) -------
    def move_l(self, params: dict[str, Any]) -> dict[str, Any]:
        pose = [float(v) for v in params["pose"]]
        speed = params.get("speed")
        speed = float(speed) if speed is not None else None
        relative = bool(params.get("relative", False))
        return self._op(
            lambda r: r.move_l(pose, speed=speed, blocking=False, relative=relative),
            gated=True,
        )

    def move_j(self, params: dict[str, Any]) -> dict[str, Any]:
        joints = [float(v) for v in params["joints"]]
        speed = params.get("speed")
        speed = float(speed) if speed is not None else None
        return self._op(lambda r: r.move_j(joints, speed=speed, blocking=False), gated=True)

    def stop(self) -> dict[str, Any]:
        return self._op(lambda r: r.stop(), gated=False)  # emergency: never gated

    def home(self) -> dict[str, Any]:
        return self._op(lambda r: r.home(blocking=False), gated=True)


# Route table: GET path -> service method; POST path -> (method, needs_body)
_GET_ROUTES = {"/api/status": "status", "/api/state": "state"}
_POST_ROUTES = {
    "/api/connect": ("connect", True),
    "/api/disconnect": ("disconnect", False),
    "/api/mode": ("set_mode", True),
    "/api/move_l": ("move_l", True),
    "/api/move_j": ("move_j", True),
    "/api/stop": ("stop", False),
    "/api/home": ("home", False),
}


def make_handler(service: RobotService, html_path: Path = _HTML_PATH):
    """Build a request handler class bound to ``service`` and the HTML file."""

    class Handler(BaseHTTPRequestHandler):
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
            except Exception as exc:
                self._send_json({"ok": False, "error": str(exc), "type": type(exc).__name__}, 400)

    return Handler


def run(host: str = "127.0.0.1", port: int = 8080) -> None:
    """Start the standalone control-panel server (blocking) on ``host:port``."""
    service = RobotService()
    httpd = ThreadingHTTPServer((host, port), make_handler(service))
    url = f"http://{host}:{port}"
    print(f"ur5e_control control panel -> {url}  (GUI control, dry-run; Ctrl-C to stop)")
    logger.info("serving on %s", url)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down…")
    finally:
        httpd.shutdown()
        service.disconnect()


def serve_in_background(
    robot: UR5eRobot,
    host: str = "127.0.0.1",
    port: int = 8080,
) -> ThreadingHTTPServer:
    """Attach a live monitor GUI to an existing robot, in a background thread.

    Use this from your own Python control script to watch live state in a browser
    while *your script* drives the robot — both share the one ``robot`` (and its
    single socket), so there is no port conflict::

        robot = UR5eRobot(RobotConfig()); robot.connect()
        serve_in_background(robot)          # open http://127.0.0.1:8080
        robot.move_l([...])                 # GUI shows it live

    The server starts in **Python control mode** (browser move/jog/home are
    locked; ``STOP`` still works). Flip to "GUI control" in the page to jog
    manually for debugging. The server runs on a daemon thread and returns
    immediately.

    Args:
        robot: The already-constructed robot to monitor/share. Connect it (or
            not) from your script as usual.
        host: Bind host (default localhost).
        port: Bind port (default 8080).

    Returns:
        The running :class:`ThreadingHTTPServer` (call ``.shutdown()`` to stop).
    """
    service = RobotService(robot=robot)
    httpd = ThreadingHTTPServer((host, port), make_handler(service))
    thread = threading.Thread(target=httpd.serve_forever, name="ur5e-gui", daemon=True)
    thread.start()
    print(f"ur5e_control monitor -> http://{host}:{port}  (Python control mode)")
    logger.info("background GUI serving on http://%s:%d", host, port)
    return httpd
