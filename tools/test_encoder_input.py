"""
Diagnostic: print live encoder values from /dev/ttyUSB0 without moving any servos.

Usage:
    python3 -m pi_teleop.tools.test_encoder_input
    python3 -m pi_teleop.tools.test_encoder_input --port /dev/ttyUSB0 --channels 0 1 2 3 4 5
"""

from __future__ import annotations

import argparse
import sys
import time

from pi_teleop.encoders import PacketReader


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Print live encoder values (no servo output)")
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--channels", type=int, nargs="+", default=list(range(6)),
                   help="Packet indices to display (default: 0-5)")
    p.add_argument("--rate-hz", type=float, default=10.0,
                   help="Display update rate (not the packet rate)")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    reader = PacketReader.open(args.port, args.baud)
    print(f"Reading from {args.port} — Ctrl+C to stop")
    print(f"Channels shown: {args.channels}")
    print()

    dt = 1.0 / args.rate_hz
    next_print = time.monotonic()
    last_packet = None

    try:
        while True:
            pkt = reader.read_latest()
            if pkt is not None:
                last_packet = pkt

            now = time.monotonic()
            if now >= next_print:
                next_print += dt
                if last_packet is None:
                    print("  (no packet yet)", flush=True)
                else:
                    vals = " ".join(
                        f"ch{i}={last_packet.values[i]:+.4f}"
                        for i in args.channels
                        if i < len(last_packet.values)
                    )
                    s = reader.stats
                    print(
                        f"  seq={last_packet.sequence:5d}  {vals}"
                        f"  [ok={s.valid} crc={s.crc_fail} dc={s.disconnects}]",
                        flush=True,
                    )
            else:
                time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        reader.close()

    print("\nStopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
