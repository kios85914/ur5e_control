"""Guided REAL-ROBOT validation of the force subsystem (FT 300 bring-up).

This is the on-robot checklist for the three force behaviors, run in a safe
order from least to most aggressive:

#. **baseline** — read the streamed wrench at rest (sanity-check the FT 300),
#. **guarded move** (``cmd=5``, NOT gated) — descend slowly along ``-Z`` until
   the contact force reaches a threshold, then stop & hold. This finds the
   surface without ever entering ``force_mode``.
#. **maintain force** (``cmd=4``, gated by ``FORCE_MODE_ENABLED``) — press
   straight down with a constant force for a few seconds, reading the wrench,
   then ``end_force()``. Single-axis: only ``Z`` is compliant, ``x/y``/rotation
   stay rigid (``direction=[0,0,-1]``).

In LIVE mode it attaches the browser monitor (``robot.serve_gui()`` ->
http://127.0.0.1:8080) and pauses for confirmation before each motion phase.

.. danger::

   **This drives the real arm into a surface.** Run it only after:

   * the Robotiq FT 300 is mounted and the controller streams a sane wrench,
   * the teach pendant is in **Remote Control** mode and you hold the e-stop,
   * there is a compliant surface (foam / your hand) ~10-15 cm below the TCP,
   * the workspace limits in :class:`RobotConfig` match your cell.

   It arms ``force_mode`` by uploading the daemon with
   ``RobotConfig(force_mode_enabled=True)``. Keep the force targets modest until
   the run behaves.

Run it::

    python -m ur5e_control.examples.force_realrobot            # DRY-RUN preview
    python -m ur5e_control.examples.force_realrobot --live     # REAL robot (asks to confirm)
    python -m ur5e_control.examples.force_realrobot --live --yes   # skip the prompts

Frames & units: poses ``[x, y, z, rx, ry, rz]`` in meters/radians; ``move_l``
inputs are world frame; wrench is ``[fx, fy, fz, tx, ty, tz]`` (N, Nm) in the UR
base frame.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

# Allow ``python ur5e_control/examples/force_realrobot.py`` without an install.
_ROOT = str(Path(__file__).resolve().parents[2])
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from ur5e_control import RobotConfig, UR5eRobot

# The READY pose: a safe Cartesian pose ABOVE the surface that every phase
# returns to (with NO load) to re-zero the FT 300-S before running. UR base frame
# == world by default. Inside the default workspace x(-0.4,0.4) y(-0.565,-0.265)
# z(-0.1,0.4). Tool pointing down (rx=0, ry=-3.14, rz=0). Adjust to your cell.
_READY_POSE = [0.0, -0.35, 0.20, 0.0, -3.14, 0.0]
_READY_SPEED = 0.2          # m/s — moving to ready is free-space, can be brisk

# Phase 2 guarded move: descend along -Z until this contact force, then stop.
_GUARD_DIR = [0.0, 0.0, -1.0]
# Overshoot ~= contact_stiffness * speed * loop_latency. Keep the speed low and
# poll fast so the PC-side stop lands near the threshold (a stiff surface still
# overshoots a few N at higher speed; for <1 N you'd need a daemon-side stop).
_GUARD_SPEED = 0.01          # m/s — slow approach (halved to cut overshoot)
_GUARD_POLL_S = 0.005        # s — fast wrench polling (was 0.02) to detect sooner
_GUARD_FORCE_N = 10.0        # N — stop at this contact force
_GUARD_MAX_TRAVEL = 0.12     # m — give up (and stop) past this

# Phase 3 maintain force: press straight down with a constant force. It starts
# from the READY pose, so force_mode first descends (at speed_limit) until it
# feels the surface, then regulates the force — give the hold enough time for
# both the descent and the hold.
_PRESS_DIR = [0.0, 0.0, -1.0]   # single-axis: only Z compliant
_PRESS_FORCE_N = 10.0           # N — keep modest for bring-up
_PRESS_SPEED_LIMIT = 0.03       # m/s — compliant-axis speed cap (descent speed)
_PRESS_MAX_TRAVEL = 0.05        # m — compliant-axis travel cap
_PRESS_HOLD_S = 8.0             # s — descent-from-ready + hold (tune to your gap)
_PRESS_POLL_S = 0.5            # s — wrench print period while holding


def _show_wrench(robot: UR5eRobot, label: str) -> None:
    """Print the latest wrench — the real FT 300-S if available, else get_tcp_force."""
    ft = getattr(robot, "ft300", None)
    if ft is not None:
        try:
            w = ft.read()  # real FT 300-S, software-tared after _zero_ft300()
            print(f"   {label}: FT300={[round(v, 2) for v in w]} (N,Nm)")
            return
        except ValueError:
            pass  # not streaming yet -> fall back below
    try:
        s = robot.get_state()
        print(f"   {label}: tcp_z={s.tcp_pose[2]:+.4f} m  "
              f"get_tcp_force={[round(v, 2) for v in s.wrench]} (N,Nm)")
    except ValueError as exc:
        print(f"   {label}: (no live state: {exc})")


def _zero_ft300(robot: UR5eRobot) -> None:
    """Software-tare the FT 300-S at the CURRENT (no-load) pose, if available.

    Call this with no external load, in the orientation you'll measure in. After
    it, ``robot.ft300.read()`` reads ~0 and the residual offset is gone. Re-call
    it whenever you return to this pose to defeat drift.
    """
    ft = getattr(robot, "ft300", None)
    if ft is None:
        print("   [dry-run / FT300 off] would zero the FT 300-S here.")
        return
    if not ft.wait_for_data(timeout=5.0):
        print("   [!] FT 300-S not streaming (check port 63351) — skipping zero.")
        return
    ft.zero()
    print("   FT 300-S zeroed — current no-load reading is the new baseline.")


def _goto_ready_and_zero(robot: UR5eRobot, live: bool, blocking: bool) -> None:
    """Return to the READY pose (no load) and software-tare the FT 300-S.

    Run at the start of every motion phase so each one begins from the same
    no-load reference: move up to ``_READY_POSE`` (free space), then zero. This
    defeats the FT 300-S drift/residual-offset between phases.
    """
    print(f"   -> ready pose {_READY_POSE}, then zero the FT 300-S (no load)")
    if live:
        robot.move_l(_READY_POSE, speed=_READY_SPEED, blocking=blocking)
        _zero_ft300(robot)
        _show_wrench(robot, "ready (zeroed)")
    else:
        print("   [dry-run] would move to ready pose and zero the FT 300-S.")


def _confirm(skip_prompt: bool) -> bool:
    """Block for an explicit 'go' before moving the real arm (unless --yes)."""
    if skip_prompt:
        return True
    print("\n*** LIVE MODE — the arm WILL move and press into a surface. ***")
    print("    Hold the e-stop. Type 'go' (then Enter) to proceed, anything else aborts.")
    try:
        return input("    > ").strip().lower() == "go"
    except EOFError:
        return False


def main(dry_run: bool = True, skip_prompt: bool = False) -> None:
    """Run the force-subsystem bring-up checklist.

    In ``dry_run`` (the default) nothing is uploaded or moved: each phase prints
    what it *would* do, so the script is safe to read/import/preview with no
    robot. With ``dry_run=False`` it uploads the daemon with ``force_mode``
    armed and executes the phases on the real controller.

    Args:
        dry_run: If ``True``, preview only (no sockets, no motion). If ``False``,
            talk to the real robot at ``config.controller_ip``.
        skip_prompt: If ``True``, do not ask for interactive confirmation before
            moving in live mode (the ``--yes`` flag).
    """
    live = not dry_run
    blocking = live  # no state stream to converge against in dry-run
    # Arm force_mode (cmd 4) on the uploaded daemon, AND read the real Robotiq
    # FT 300/FT 300-S from the URCap stream (port 63351) so guarded_move triggers
    # on the actual sensor and we can zero it. Harmless in dry-run (no sockets).
    config = RobotConfig(force_mode_enabled=True, ft300_enabled=True)

    mode = "LIVE (real robot)" if live else "DRY-RUN (nothing moves)"
    print(f"== UR5e force-subsystem bring-up — {mode} ==")
    print(f"controller_ip={config.controller_ip}  pc_host={config.pc_host}  "
          f"force_mode_enabled={config.force_mode_enabled}  "
          f"ft300_enabled={config.ft300_enabled}\n")

    if live and not _confirm(skip_prompt):
        print("Aborted (no confirmation).")
        return

    with UR5eRobot(config, dry_run=dry_run) as robot:
        if robot.wait_until_connected(timeout=8.0):
            print("[OK] daemon dialed back; state is streaming.\n")
        elif dry_run:
            print("[dry-run] no real robot (preview only).\n")
        else:
            print("[!] robot NOT connected in 8 s — check Remote Control / IPs / "
                  f"firewall on state port {config.state_port}. Aborting.\n")
            return

        # Attach the browser monitor for the live run (gated: dry-run stays
        # socket-free so the preview/tests open nothing).
        if live:
            robot.serve_gui()
            print("GUI monitor: http://127.0.0.1:8080\n")

        # --- Phase 1: baseline wrench -------------------------------------
        print("1) baseline wrench at rest (sanity-check the FT 300)")
        _show_wrench(robot, "rest")

        # --- Phase 2: guarded move (each phase: ready+zero, then run) ------
        print(f"\n2) guarded move: descend {_GUARD_DIR} at {_GUARD_SPEED} m/s "
              f"until {_GUARD_FORCE_N} N, then stop & hold")
        if live and not _confirm(skip_prompt):
            print("Aborted (no confirmation).")
            return
        _goto_ready_and_zero(robot, live, blocking)
        if live:
            contact = robot.force.guarded_move(
                direction=_GUARD_DIR,
                speed=_GUARD_SPEED,
                force_threshold_n=_GUARD_FORCE_N,
                max_travel=_GUARD_MAX_TRAVEL,
                poll_interval=_GUARD_POLL_S,   # fast poll -> less overshoot
            )
            print(f"   contact wrench: {[round(v, 2) for v in contact]} (N,Nm)")
            _show_wrench(robot, "holding contact")
        else:
            print("   [dry-run] would speedl down and stop at the threshold "
                  "(cmd 5 -> cmd 2; not gated).")

        # --- Phase 3: maintain force (ready+zero, then press) -------------
        print(f"\n3) maintain force: press {_PRESS_DIR} at constant {_PRESS_FORCE_N} N "
              f"for {_PRESS_HOLD_S} s (single-axis: only Z compliant)")
        if live and not _confirm(skip_prompt):
            print("Aborted (no confirmation).")
            return
        _goto_ready_and_zero(robot, live, blocking)
        if live:
            robot.force.maintain_force(
                direction=_PRESS_DIR,
                target_n=_PRESS_FORCE_N,
                speed_limit=_PRESS_SPEED_LIMIT,
                max_travel=_PRESS_MAX_TRAVEL,
            )
            t_end = time.monotonic() + _PRESS_HOLD_S
            while time.monotonic() < t_end:
                _show_wrench(robot, "pressing")
                time.sleep(_PRESS_POLL_S)
            robot.force.end_force()
            print("   end_force() sent — arm holds its pose stiffly.")
            _show_wrench(robot, "after end_force")
        else:
            print("   [dry-run] would enter force_mode (cmd 4, armed via "
                  "force_mode_enabled=True) and hold the force, then cmd 6.")

        # --- Phase 4: back to ready, then home ---------------------------
        print("\n4) return to ready pose, then home")
        if live:
            _goto_ready_and_zero(robot, live, blocking)
            robot.home(speed=0.1, blocking=blocking)
        else:
            print("   [dry-run] would return to ready, then home.")

    print("\nDone. (Context manager disconnected the robot.)")
    if not live:
        print("Re-run with --live (and --yes to skip the prompts) on the real robot.")


if __name__ == "__main__":
    args = sys.argv[1:]
    main(dry_run="--live" not in args, skip_prompt="--yes" in args)
