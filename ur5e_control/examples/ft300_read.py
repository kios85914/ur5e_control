"""Read the REAL Robotiq FT 300 / FT 300-S from the URCap stream (TCP port 63351).

Why this exists: the daemon's state stream carries URScript ``get_tcp_force()``,
which is the UR's OWN force value (on a CB-series UR3 it is a joint-torque
*estimate*, not a sensor) — it is NOT the external Robotiq FT 300/FT 300-S. The
FT 300-S is published by the Robotiq URCap on the controller's TCP port 63351.
This script reads that port directly from the PC (over the existing network link
to the controller — no extra cable).

It runs in two phases:
  1. RAW: print the first few bytes verbatim, so you can SEE the exact wire
     format (we parse what's really there instead of guessing).
  2. PARSED: print the live wrench [fx, fy, fz, tx, ty, tz] until Ctrl-C.

Sanity check while it runs: place a ~400 g object on the tool — the change should
be about 3.9 N (m*g), NOT ~10 N. If you see ~3.9 N here, the real sensor path
works. (Zero the sensor first in PolyScope / via the URCap, no load.)

Run (on a PC that can reach the UR controller):
    python -m ur5e_control.examples.ft300_read --host 192.168.0.137
"""

from __future__ import annotations

import socket
import sys
import time
from pathlib import Path

_ROOT = str(Path(__file__).resolve().parents[2])
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from ur5e_control import RobotConfig
from ur5e_control.force.sensor import RobotiqFT300Stream

_DEFAULT_PORT = 63351


def main(host: str | None = None, port: int = _DEFAULT_PORT,
         raw_reads: int = 8, duration: float = 20.0, do_zero: bool = False) -> None:
    """Probe + stream the Robotiq FT sensor on ``host:port``.

    Args:
        host: UR controller IP. ``None`` uses ``RobotConfig().controller_ip``.
        port: Robotiq stream port (default 63351).
        raw_reads: How many raw recv() chunks to dump in phase 1.
        duration: Seconds to print parsed wrench in phase 2.
        do_zero: If ``True`` (``--zero``), software-tare at the start (ensure NO
            load) so readings begin near 0 — then a ~400 g object should read
            ~3.9 N. Re-zeroing each time you return to this pose defeats drift.
    """
    host = host or RobotConfig().controller_ip
    print(f"== Robotiq FT 300-S reader — {host}:{port} ==\n")

    # --- Phase 1: raw, to reveal the exact wire format ----------------------
    print("1) RAW bytes from the stream (confirm the format):")
    try:
        sock = socket.create_connection((host, port), timeout=5.0)
    except OSError as exc:
        print(f"   [FAIL] could not connect to {host}:{port} ({exc}).")
        print("   Check: FT sensor wired + powered, Robotiq URCap installed and")
        print("   showing 'connected' on the pendant, PC on the same network.")
        return
    try:
        sock.settimeout(2.0)
        for _ in range(raw_reads):
            try:
                chunk = sock.recv(256)
            except socket.timeout:
                print("   (timeout waiting for data)")
                break
            if not chunk:
                print("   (stream closed)")
                break
            print("  ", repr(chunk))
    finally:
        sock.close()

    # --- Phase 2: parsed wrench via the library reader ---------------------
    print("\n2) PARSED wrench [fx, fy, fz, tx, ty, tz] (Ctrl-C to stop):")
    sensor = RobotiqFT300Stream(host, port)
    sensor.start()
    try:
        if not sensor.wait_for_data(timeout=5.0):
            print("   [FAIL] connected but no parseable 6-value record arrived.")
            print("   Paste the RAW output above so the parser can be matched.")
            return
        if do_zero:
            print("   software-taring now (make sure there is NO load)...")
            sensor.zero()
            print("   tared; readings below are relative to this baseline.")
        t_end = time.monotonic() + duration
        while time.monotonic() < t_end:
            w = sensor.read()
            print("   " + "  ".join(f"{v:+8.3f}" for v in w))
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        sensor.close()
    print("\nDone.")


if __name__ == "__main__":
    args = sys.argv[1:]
    host = None
    if "--host" in args:
        host = args[args.index("--host") + 1]
    main(host=host, do_zero="--zero" in args)
