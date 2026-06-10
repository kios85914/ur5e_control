"""Robot state model and parser for the UR5e control library.

The daemon streams the robot state back to the PC as ASCII strings. Each frame
uses the documented format (all values in **meters / radians**, expressed in the
**UR base frame**)::

    p[x,y,z,rx,ry,rz]_p[vx,vy,vz,wx,wy,wz]_[j0,j1,j2,j3,j4,j5]_p[fx,fy,fz,tx,ty,tz]+

Groups, in order, are:

* ``p[...]`` TCP pose       — ``[x, y, z, rx, ry, rz]`` (m, rad), UR base frame
* ``p[...]`` TCP speed      — ``[vx, vy, vz, wx, wy, wz]`` (m/s, rad/s)
* ``[...]``  joint angles   — ``[j0, j1, j2, j3, j4, j5]`` (rad)
* ``p[...]`` TCP wrench     — ``[fx, fy, fz, tx, ty, tz]`` (N, Nm)

Frames are delimited by ``+``. Several frames may arrive concatenated in one
read; :func:`parse_state` always uses the **last complete** frame.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

__all__ = ["RobotState", "parse_state"]

# Number of scalar values expected in each state group.
_GROUP_LEN = 6


@dataclass
class RobotState:
    """A single snapshot of the UR5e robot state.

    All quantities are expressed in the **UR base frame**; lengths are in
    **meters** and angles in **radians** (velocities in m/s and rad/s, forces in
    N and torques in Nm). Field order is part of the locked interface and must
    not change.

    Attributes:
        tcp_pose: TCP pose ``[x, y, z, rx, ry, rz]`` in meters and radians.
        tcp_speed: TCP speed ``[vx, vy, vz, wx, wy, wz]`` in m/s and rad/s.
        joints: Joint angles ``[j0, j1, j2, j3, j4, j5]`` in radians.
        wrench: TCP wrench ``[fx, fy, fz, tx, ty, tz]`` in N and Nm.
        timestamp: ``time.time()`` value captured when the frame was parsed (s).
    """

    tcp_pose: list[float]
    tcp_speed: list[float]
    joints: list[float]
    wrench: list[float]
    timestamp: float


def _parse_group(group: str, expect_p: bool) -> list[float]:
    """Parse one ``p[...]`` or ``[...]`` group into a 6-element float list.

    Args:
        group: The raw group token, e.g. ``"p[1,2,3,4,5,6]"`` or ``"[1,2,3,4,5,6]"``.
        expect_p: ``True`` if the group must start with the ``p`` prefix
            (pose/speed/wrench), ``False`` for a bare bracket group (joints).

    Returns:
        A list of exactly six floats.

    Raises:
        ValueError: If the prefix/brackets are malformed, the element count is
            not six, or any element is not a valid float.
    """
    body = group.strip()
    if expect_p:
        if not body.startswith("p["):
            raise ValueError(f"Expected a 'p[...]' group, got: {group!r}")
        body = body[1:]  # drop the leading 'p', leaving '[...]'
    if not (body.startswith("[") and body.endswith("]")):
        raise ValueError(f"Malformed bracketed group: {group!r}")

    inner = body[1:-1].strip()
    if not inner:
        raise ValueError(f"Empty state group: {group!r}")

    parts = [p.strip() for p in inner.split(",")]
    if len(parts) != _GROUP_LEN:
        raise ValueError(
            f"Expected {_GROUP_LEN} values in group {group!r}, got {len(parts)}"
        )

    try:
        return [float(p) for p in parts]
    except ValueError as exc:
        raise ValueError(f"Non-numeric value in group {group!r}: {exc}") from exc


def parse_state(raw: str) -> RobotState:
    """Parse a raw daemon state string into a :class:`RobotState`.

    The input may contain several ``+``-delimited frames; the **last complete**
    frame (the last one terminated by ``+``) is parsed. A trailing partial frame
    (text after the final ``+``) is ignored. Parsed poses, speeds, joints and
    wrenches are in the **UR base frame** (meters / radians).

    Args:
        raw: One or more concatenated state frames in the documented format.

    Returns:
        A :class:`RobotState` stamped with ``timestamp = time.time()``.

    Raises:
        ValueError: If ``raw`` is not a string, contains no complete frame, or
            the last complete frame is malformed (wrong group count, bad
            prefixes/brackets, wrong element count, or non-numeric values).
    """
    if not isinstance(raw, str):
        raise ValueError(f"State must be a string, got {type(raw).__name__}")

    # Each complete frame ends with '+'. Split and keep everything before the
    # final '+' so any trailing partial frame is discarded.
    frames = raw.split("+")
    if len(frames) < 2:
        # No '+' at all -> no complete frame was received.
        raise ValueError("No complete state frame (missing '+' terminator)")

    # frames[-1] is whatever followed the last '+'; the last COMPLETE frame is
    # the last non-empty element among frames[:-1].
    frame = ""
    for candidate in reversed(frames[:-1]):
        if candidate.strip():
            frame = candidate.strip()
            break
    if not frame:
        raise ValueError("No non-empty complete state frame found")

    groups = frame.split("_")
    if len(groups) != 4:
        raise ValueError(
            f"Expected 4 underscore-delimited groups, got {len(groups)}: {frame!r}"
        )

    tcp_pose = _parse_group(groups[0], expect_p=True)
    tcp_speed = _parse_group(groups[1], expect_p=True)
    joints = _parse_group(groups[2], expect_p=False)
    wrench = _parse_group(groups[3], expect_p=True)

    return RobotState(
        tcp_pose=tcp_pose,
        tcp_speed=tcp_speed,
        joints=joints,
        wrench=wrench,
        timestamp=time.time(),
    )
