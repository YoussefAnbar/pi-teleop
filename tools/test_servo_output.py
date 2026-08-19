"""
Diagnostic: send a single goal position to one servo without reading any encoders.
Use this to verify /dev/serial0 wiring and servo IDs before running the full bridge.

Usage:
    # Sweep servo ID 1 from -0.5 to +0.5 rad and back
    python3 -m pi_teleop.tools.test_servo_output --id 1

    # Send a specific position (rad) to servo ID 3
    python3 -m pi_teleop.tools.test_servo_output --id 3 --pos 0.3

    # Disable torque on all servos (go compliant)
    python3 -m pi_teleop.tools.test_servo_output --torque-off
"""

from __future__ import annotations

import argparse
import math
import sys
import time

from pi_teleop.robot_interfaces import SO101Bus


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Single-servo test (no encoder input)")
    p.add_argument("--port", default="/dev/serial0")
    p.add_argument("--baud", type=int, default=1_000_000)
    p.add_argument("--id", type=int, default=1, dest="servo_id",
                   help="Servo ID to command (1-6)")
    p.add_argument("--pos", type=float, default=None,
                   help="Target position in radians (if omitted, does a sweep)")
    p.add_argument("--torque-off", action="store_true",
                   help="Disable torque on ALL servos and exit")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    bus = SO101Bus(port=args.port, baud=args.baud)

    if args.torque_off:
        print("Disabling torque on all servos...")
        bus.torque_enable_all(False)
        bus.close()
        print("Done — arm is now compliant.")
        return 0

    print(f"Enabling torque on servo {args.servo_id}...")
    bus.torque_enable(args.servo_id, True)
    time.sleep(0.1)

    try:
        if args.pos is not None:
            print(f"  Sending {args.pos:.3f} rad to servo {args.servo_id}")
            bus.set_goal_position_rad(args.servo_id, args.pos)
            time.sleep(1.0)
        else:
            print(f"  Sweeping servo {args.servo_id} from -0.5 to +0.5 rad (Ctrl+C to stop)...")
            t = 0.0
            while True:
                pos = 0.5 * math.sin(t)
                bus.set_goal_position_rad(args.servo_id, pos)
                print(f"  pos={pos:+.3f} rad", end="\r", flush=True)
                time.sleep(0.02)
                t += 0.02 * 0.5  # ~0.5 Hz sine
    except KeyboardInterrupt:
        print()
    finally:
        print(f"Disabling torque on servo {args.servo_id}...")
        bus.torque_enable(args.servo_id, False)
        bus.close()

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
