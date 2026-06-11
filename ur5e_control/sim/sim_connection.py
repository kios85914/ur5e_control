"""A RobotConnection-compatible transport backed by the MuJoCo UR5e sim.

Drop-in for :class:`~ur5e_control.connection.RobotConnection`: it parses the same
ASCII command tuples the daemon would receive and drives a :class:`MujocoUR5e`,
and returns state frames in the same ``p[...]_..._+`` wire format. Inject it into
the facade::

    from ur5e_control import UR5eRobot, RobotConfig
    from ur5e_control.sim import SimConnection

    robot = UR5eRobot(RobotConfig(), connection=SimConnection(viewer=True))
    with robot:
        robot.wait_until_connected()
        robot.move_j([0, -1.2, 1.2, -1.57, -1.57, 0])
        contact = robot.force.guarded_move([0, 0, -1], speed=0.03,
                                           force_threshold_n=20.0, max_travel=0.15)
"""

from __future__ import annotations

from pathlib import Path

from .mujoco_backend import MujocoUR5e

__all__ = ["SimConnection"]


class SimConnection:
    """Sim transport: ``send``/``latest_state``/``start``/``close``/``is_connected``."""

    def __init__(self, scene_path: str | Path | None = None, realtime: bool = True,
                 viewer: bool = False) -> None:
        kwargs = {"realtime": realtime, "viewer": viewer}
        if scene_path is not None:
            kwargs["scene_path"] = scene_path
        self._sim = MujocoUR5e(**kwargs)
        self._started = False

    def start(self) -> None:
        self._sim.start()
        self._started = True

    def send(self, msg: str) -> None:
        nums = [float(x) for x in msg.strip().strip("()").split(",")]
        if len(nums) != 10:
            raise ValueError(f"expected 10-float command tuple, got {len(nums)}: {msg!r}")
        cmd = int(round(nums[0]))
        self._sim.command(cmd, nums[1:7], nums[7], nums[8], nums[9])

    def latest_state(self) -> str:
        return self._sim.latest_state()

    def is_connected(self) -> bool:
        # True once stepping has produced at least one state frame, so
        # wait_until_connected() blocks until the sim is actually streaming.
        return self._started and bool(self._sim.latest_state())

    def close(self) -> None:
        self._sim.close()
        self._started = False

    def __enter__(self) -> "SimConnection":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.close()
