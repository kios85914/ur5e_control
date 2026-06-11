"""MuJoCo simulation backend for ur5e_control (PC-side testing, no hardware).

The library's transport is abstracted (``RobotConnection``: ``send(str)`` +
``latest_state() -> str``). :class:`~ur5e_control.sim.sim_connection.SimConnection`
implements that interface backed by a MuJoCo simulation of the **official
mujoco_menagerie UR5e** (accurate kinematics/inertia/meshes) plus a table and a
wrist force/torque sensor. So the *same* ``MotionController`` / ``ForceController``
/ ``guarded_move`` code runs against physics instead of the real controller::

    from ur5e_control import UR5eRobot, RobotConfig
    from ur5e_control.sim import SimConnection

    sim = SimConnection(viewer=True)
    robot = UR5eRobot(RobotConfig(), connection=sim)   # inject the sim transport
    with robot:
        robot.wait_until_connected()
        contact = robot.force.guarded_move([0, 0, -1], speed=0.03,
                                           force_threshold_n=20.0, max_travel=0.15)

Caveat: MuJoCo does NOT execute URScript. ``moveL/moveJ/speedl`` are reproduced
by a Jacobian controller, and the force-mode behaviors (cmd 4/7) are reproduced
by a simple admittance law in the sim — so this validates the library's control
logic + contact physics, NOT the shipped ``motion_daemon.script`` (use URSim for
that). Requires ``mujoco`` (``uv pip install mujoco``).
"""

from .sim_connection import SimConnection

__all__ = ["SimConnection"]
