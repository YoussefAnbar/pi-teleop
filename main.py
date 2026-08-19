"""
Pi-side teleop loop:  ESP32 encoder packet  ->  channel mapper  ->  SO-101 servos.

Usage (on the Pi):
    python3 -m pi_teleop.main
    python3 -m pi_teleop.main --encoder-port /dev/ttyUSB0 --servo-port /dev/serial0 --dry-run

Dependencies: pyserial only.  No DDS, no PyTorch, no MuJoCo.
"""

from __future__ import annotations

import argparse
import importlib
import signal
import sys
import time

from pi_teleop.encoders import PacketReader
from pi_teleop.mapping import BridgeConfig, ChannelMapper
from pi_teleop.robot_interfaces import SO101Bus


def load_config(module_path: str) -> BridgeConfig:
    module = importlib.import_module(module_path)
    config = getattr(module, "BRIDGE_CONFIG", None)
    if not isinstance(config, BridgeConfig):
        raise SystemExit(f"{module_path} does not export a BRIDGE_CONFIG: BridgeConfig")
    return config


def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="SO-101 Pi teleop loop")
    p.add_argument("--encoder-port", default="/dev/ttyUSB0",
                   help="Serial port for the ESP32 encoder stream (CP2102 bridge)")
    p.add_argument("--encoder-baud", type=int, default=115200)
    p.add_argument("--servo-port", default="/dev/serial0",
                   help="UART to the Waveshare Bus Servo Adapter (SO-101 follower)")
    p.add_argument("--servo-baud", type=int, default=1_000_000,
                   help="Feetech default is 1 Mbaud")
    p.add_argument("--config", default="pi_teleop.configs.so101_default",
                   help="Python module that exports BRIDGE_CONFIG")
    p.add_argument("--rate-hz", type=float, default=100.0)
    p.add_argument("--dry-run", action="store_true",
                   help="Read + map packets but do NOT open the servo bus")
    p.add_argument("--print-every", type=float, default=2.0,
                   help="Seconds between stats prints (0 disables)")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    config = load_config(args.config)
    mapper = ChannelMapper(config)

    print(f"[pi_teleop] opening encoder port {args.encoder_port} @ {args.encoder_baud}")
    reader = PacketReader.open(args.encoder_port, args.encoder_baud)

    bus: SO101Bus | None = None
    if not args.dry_run:
        servo_ids = tuple(b.servo_id for b in config.joint_bindings)
        print(f"[pi_teleop] opening servo port {args.servo_port} @ {args.servo_baud}")
        bus = SO101Bus(port=args.servo_port, baud=args.servo_baud, servo_ids=servo_ids)

    stop = {"v": False}

    def _sigint(_sig, _frm):
        stop["v"] = True
    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    print(f"[pi_teleop] config={config.name}  bindings={len(config.joint_bindings)}  "
          f"dry_run={args.dry_run}  rate={args.rate_hz} Hz")
    print("[pi_teleop] waiting for first encoder packet to home...")

    dt = 1.0 / args.rate_hz
    next_tick = time.monotonic()
    last_print = time.monotonic()
    ticks_with_packet = 0
    ticks_total = 0
    torque_armed = False

    try:
        while not stop["v"]:
            ticks_total += 1
            packet = reader.read_latest()
            if packet is not None:
                targets = mapper.apply(packet.values)
                if targets and bus is not None:
                    if not torque_armed:
                        # Enable torque only after the first homed targets are computed.
                        # The arm should already be at a pose close to the operator's.
                        bus.torque_enable_all(True)
                        torque_armed = True
                        print("[pi_teleop] homed — torque enabled, arm is now tracking")
                    bus.write_targets(targets)
                ticks_with_packet += 1

            if args.print_every > 0 and (time.monotonic() - last_print) >= args.print_every:
                s = reader.stats
                print(
                    f"[pi_teleop] homed={mapper.is_homed} "
                    f"pkt_ticks={ticks_with_packet}/{ticks_total} "
                    f"valid={s.valid} crc_fail={s.crc_fail} invalid={s.invalid} "
                    f"stale={s.stale} desync={s.desync_bytes} "
                    f"disconnects={s.disconnects}"
                )
                last_print = time.monotonic()

            next_tick += dt
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()
    finally:
        if bus is not None and torque_armed:
            try:
                bus.torque_enable_all(False)
                print("[pi_teleop] torque disabled — arm is now compliant")
            except Exception:
                pass
        reader.close()
        if bus is not None:
            bus.close()

    print("[pi_teleop] stopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
