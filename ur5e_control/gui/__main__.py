"""Launch the ur5e_control web control panel.

Usage::

    python -m ur5e_control.gui                  # http://127.0.0.1:8080
    python -m ur5e_control.gui --port 9000
    python -m ur5e_control.gui --host 0.0.0.0   # expose on the LAN (use with care)
"""

from __future__ import annotations

import argparse
import logging

from .server import run


def main() -> None:
    parser = argparse.ArgumentParser(description="ur5e_control web control panel")
    parser.add_argument("--host", default="127.0.0.1", help="bind host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8080, help="bind port (default: 8080)")
    parser.add_argument("--verbose", action="store_true", help="debug logging")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    run(host=args.host, port=args.port)


if __name__ == "__main__":
    main()
