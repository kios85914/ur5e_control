"""MuJoCo physics backend for the UR5e (drives the menagerie model).

:class:`MujocoUR5e` loads the UR5e scene, runs physics on a background thread,
and turns the library's command tuples into joint targets via a Jacobian
controller. It reports TCP pose and the wrist wrench in the **UR base frame**
(matching the real robot's ``get_tcp_force()`` convention), so the unchanged
``MotionController`` / ``ForceController`` logic works against it.

What's faithful vs approximated:

* moveJ — exact (position actuators).
* moveL / speedl — Jacobian resolved-rate (linear + angular).
* force_mode (maintain force / impedance) — a simple *sim* admittance law (the
  real controller's 500 Hz force_mode is URScript and is not run here).
* TCP pose, joints, and the contact wrench (from the MuJoCo force/torque sensor)
  are physical.
"""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path

import numpy as np

try:
    import mujoco
except ImportError as exc:  # pragma: no cover - exercised only without mujoco
    raise ImportError(
        "ur5e_control.sim requires the 'mujoco' package: uv pip install mujoco"
    ) from exc

_SCENE = Path(__file__).resolve().parent / "assets" / "ur5e" / "scene.xml"

_JOINTS = (
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
)

# Resolved-rate gains for moveL (Cartesian target tracking).
_KP_LIN = 2.0       # 1/s on position error -> commanded linear vel
_KP_ROT = 2.0       # 1/s on orientation error
# Admittance gains for the sim force-mode reproduction.
_K_FORCE = 0.002    # m/s per N (maintain-force P gain)
_IMP_DAMP = 200.0   # N/(m/s) admittance damping for impedance


def _mat_to_rotvec(R: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> axis-angle rotation vector (rx, ry, rz)."""
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, R.flatten())
    w, xyz = quat[0], quat[1:]
    n = np.linalg.norm(xyz)
    if n < 1e-9:
        return np.zeros(3)
    angle = 2.0 * math.atan2(n, w)
    return (xyz / n) * angle


class MujocoUR5e:
    """Background-stepping MuJoCo sim of the UR5e with a Jacobian controller."""

    def __init__(self, scene_path: str | Path = _SCENE, realtime: bool = True,
                 viewer: bool = False) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(scene_path))
        self.data = mujoco.MjData(self.model)
        self._realtime = realtime
        self._want_viewer = viewer
        self._viewer = None

        m = self.model
        self._jids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in _JOINTS]
        self._qadr = [m.jnt_qposadr[j] for j in self._jids]
        self._site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
        self._base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")
        self._f_adr = m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "ft_force")]
        self._t_adr = m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "ft_torque")]
        self._jrange = m.jnt_range[self._jids].copy()

        # Start at the 'home' keyframe and hold there.
        key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "home")
        mujoco.mj_resetDataKeyframe(m, self.data, key)
        mujoco.mj_forward(m, self.data)
        self._target_q = self.data.qpos[self._qadr].copy()
        self._home_q = self._target_q.copy()
        self.data.ctrl[:] = self._target_q

        # Command state (guarded by _lock); set by command(), read by the loop.
        self._lock = threading.Lock()
        self._mode = "hold"
        self._params = {}
        self._cmd_time = time.monotonic()
        self._latest = ""

        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> None:
        if self._want_viewer:
            import mujoco.viewer
            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="mujoco-sim", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._viewer is not None:
            try:
                self._viewer.close()
            except Exception:
                pass

    # ------------------------------------------------------------------ command in
    def command(self, cmd: int, payload, accel: float, vel: float, t: float) -> None:
        """Apply a parsed command tuple (called from SimConnection.send)."""
        with self._lock:
            self._cmd_time = time.monotonic()
            if cmd == 0:      # moveL: payload = base-frame pose [x,y,z,rx,ry,rz]
                self._mode, self._params = "movel", {"pose": np.array(payload, float)}
            elif cmd == 1:    # moveJ: payload = joints
                self._mode, self._params = "movej", {"q": np.array(payload, float)}
            elif cmd == 2:    # stop / hold
                self._mode = "hold"
            elif cmd == 3:    # home -> move to the keyframe home joints
                self._mode, self._params = "movej", {"q": self._home_q}
            elif cmd == 4:    # maintain force: dir(3), target_n, speed_limit, max_travel
                self._mode = "force"
                self._params = {"dir": np.array(payload[:3], float), "target_n": payload[3],
                                "vmax": payload[4]}
            elif cmd == 5:    # speedl: payload = base-frame velocity [vx..wz]
                self._mode = "speedl"
                self._params = {"v": np.array(payload, float), "watchdog": t}
            elif cmd == 6:    # end force -> hold
                self._mode = "hold"
            elif cmd == 7:    # impedance: axes(3), stiffness, speed_limit, max_dev
                self._mode = "impedance"
                self._params = {"axes": np.array(payload[:3], float), "k": payload[3],
                                "vmax": payload[4], "x_eq": self._tcp_pose_base()[0].copy()}

    def latest_state(self) -> str:
        with self._lock:
            return self._latest

    # ------------------------------------------------------------------ helpers
    def _base_rot(self) -> np.ndarray:
        return self.data.xmat[self._base].reshape(3, 3)

    def _tcp_pose_base(self):
        """Return (pos[3], rotvec[3]) of the TCP in the UR base frame."""
        Rwb = self._base_rot()
        p_world = self.data.site_xpos[self._site] - self.data.xpos[self._base]
        p_base = Rwb.T @ p_world
        R_tcp_base = Rwb.T @ self.data.site_xmat[self._site].reshape(3, 3)
        return p_base, _mat_to_rotvec(R_tcp_base)

    def _wrench_base(self) -> np.ndarray:
        """Wrist force/torque expressed in the UR base frame [fx,fy,fz,tx,ty,tz]."""
        Rwb = self._base_rot()
        Rws = self.data.site_xmat[self._site].reshape(3, 3)
        f_site = self.data.sensordata[self._f_adr:self._f_adr + 3]
        t_site = self.data.sensordata[self._t_adr:self._t_adr + 3]
        f_base = Rwb.T @ (Rws @ f_site)
        t_base = Rwb.T @ (Rws @ t_site)
        return np.concatenate([f_base, t_base])

    def _jac6(self) -> np.ndarray:
        jp = np.zeros((3, self.model.nv)); jr = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jp, jr, self._site)
        return np.vstack([jp[:, self._jids], jr[:, self._jids]])  # 6x6

    def _integrate_twist_base(self, twist_base: np.ndarray, dt: float) -> None:
        """Map a base-frame twist [v(3), w(3)] to joint targets via the Jacobian."""
        Rwb = self._base_rot()
        twist_world = np.concatenate([Rwb @ twist_base[:3], Rwb @ twist_base[3:]])
        qd = np.linalg.pinv(self._jac6()) @ twist_world
        self._target_q = np.clip(self._target_q + qd * dt, self._jrange[:, 0], self._jrange[:, 1])

    # ------------------------------------------------------------------ step loop
    def _run(self) -> None:
        m, d = self.model, self.data
        dt = m.opt.timestep
        while not self._stop.is_set():
            t0 = time.monotonic()
            with self._lock:
                self._control(dt)
                d.ctrl[:] = self._target_q
            mujoco.mj_step(m, d)
            with self._lock:
                self._latest = self._format_state()
            if self._viewer is not None:
                try:
                    self._viewer.sync()
                except Exception:
                    pass
            if self._realtime:
                lag = dt - (time.monotonic() - t0)
                if lag > 0:
                    time.sleep(lag)

    def _control(self, dt: float) -> None:
        mode, p = self._mode, self._params
        if mode == "movej":
            self._target_q = np.clip(p["q"], self._jrange[:, 0], self._jrange[:, 1])
        elif mode == "movel":
            pos, rot = self._tcp_pose_base()
            perr = np.clip((p["pose"][:3] - pos) * _KP_LIN, -0.5, 0.5)
            # orientation error via quaternion difference (target vs current)
            werr = self._orient_err_base(p["pose"][3:]) * _KP_ROT
            self._integrate_twist_base(np.concatenate([perr, werr]), dt)
        elif mode == "speedl":
            if time.monotonic() - self._cmd_time > p.get("watchdog", 1.0):
                self._mode = "hold"            # watchdog: stop if not refreshed
            else:
                self._integrate_twist_base(p["v"], dt)
        elif mode == "force":
            f_along = float(self._wrench_base()[:3] @ p["dir"])
            v = np.clip(_K_FORCE * (p["target_n"] - f_along), -p["vmax"], p["vmax"])
            self._integrate_twist_base(np.concatenate([p["dir"] * v, np.zeros(3)]), dt)
        elif mode == "impedance":
            pos, _ = self._tcp_pose_base()
            f = self._wrench_base()[:3]
            v = (f - p["k"] * (pos - p["x_eq"])) / _IMP_DAMP * p["axes"]
            v = np.clip(v, -p["vmax"], p["vmax"])
            self._integrate_twist_base(np.concatenate([v, np.zeros(3)]), dt)
        # "hold": leave _target_q unchanged

    def _orient_err_base(self, target_rotvec: np.ndarray) -> np.ndarray:
        """Angular error (base frame) from current TCP orientation to a target rotvec."""
        Rwb = self._base_rot()
        R_cur = Rwb.T @ self.data.site_xmat[self._site].reshape(3, 3)
        q_cur = np.zeros(4); mujoco.mju_mat2Quat(q_cur, R_cur.flatten())
        ang = np.linalg.norm(target_rotvec)
        q_tgt = np.zeros(4)
        axis = target_rotvec / ang if ang > 1e-9 else np.array([1.0, 0, 0])
        mujoco.mju_axisAngle2Quat(q_tgt, axis, ang)
        dq = np.zeros(3)
        mujoco.mju_subQuat(dq, q_tgt, q_cur)  # angular velocity from cur -> tgt
        return dq

    def _format_state(self) -> str:
        pos, rot = self._tcp_pose_base()
        pose = np.concatenate([pos, rot])
        q = self.data.qpos[self._qadr]
        vel6 = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_SITE,
                                 self._site, vel6, 0)
        Rwb = self._base_rot()
        speed = np.concatenate([Rwb.T @ vel6[3:], Rwb.T @ vel6[:3]])  # [lin, ang] base
        wr = self._wrench_base()
        def g(a):
            return ",".join(f"{v:.6f}" for v in a)
        return f"p[{g(pose)}]_p[{g(speed)}]_[{g(q)}]_p[{g(wr)}]+"
