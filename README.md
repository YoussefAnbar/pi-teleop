# pi_teleop

Lightweight Raspberry Pi 4 runtime that maps ESP32/MCP3008 pot and encoder input to an SO-101 follower arm over `/dev/serial0` (Waveshare Bus Servo Adapter, UART mode A).

No PyTorch, no DDS, no MuJoCo, no GUI. Just Python and `pyserial`.

## Layout

```
pi_teleop/
├── main.py                         # read -> map -> write loop
├── encoders/
│   └── mcp3008_serial.py           # CRC16 24-float packet reader
├── mapping/
│   └── channel_to_joint.py         # JointBinding + scale/offset/rate-limit
├── robot_interfaces/
│   └── so101_serial.py             # Feetech STS3215 goal-position writer
├── configs/
│   └── so101_default.py            # 6-DoF default binding template
├── firmware_ref/                   # ESP32 reference only, not built on Pi
└── requirements.txt                # pyserial>=3.5
```

## Install on the Pi

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

Dry run, no servo writes. Useful for confirming the ESP32 stream is valid:

```bash
python3 -m pi_teleop.main --dry-run
```

Live:

```bash
python3 -m pi_teleop.main \
  --encoder-port /dev/ttyACM0 \
  --servo-port /dev/serial0 \
  --config pi_teleop.configs.so101_default
```

## Bring-up checklist

1. Power the SO-101 from its external servo supply, not the Pi.
2. Confirm the Waveshare adapter is on UART mode A and visible at `/dev/serial0`.
3. ESP32 should enumerate as `/dev/ttyACM0`. Adjust `--encoder-port` if not.
4. Hold the arm in the pose you want to start from and run `--dry-run` first to check for valid packets (`homed=True` and a rising `valid=` counter).
5. Drop `--dry-run`. The first packet captures home, so the arm matches the operator pose at that instant and tracks deltas from there.

## Tuning

Edit `configs/so101_default.py`:

- `scale` sign sets the direction of each joint.
- `min_q` / `max_q` are software joint limits in radians.
- `max_dq` is the rad/s rate limit, a safety cap on motion speed.
- `max_angle_jump` rejects one-tick spikes from noisy pots.

## Attribution

Some of the mapping and configuration approach here comes from [`matthehzhang/inhabit_teleop`](https://github.com/matthehzhang/inhabit_teleop), a team project I contributed to. The channel-to-joint mapping structure follows that project's `g1_bridge_lib_20ch.py`, and the per-joint configuration shape follows its `g1_POTCONFIG.py`.

File by file:

- `mapping/channel_to_joint.py` comes from `programs/g1_bridge_lib_20ch.py` mapping math, with all Unitree DDS / HG_LowCmd / CycloneDDS imports stripped.
- `configs/so101_default.py` comes from `programs/g1_POTCONFIG.py`.
- `encoders/mcp3008_serial.py` comes from `programs/scratch/read_mcp3008_packets.py`. The wire format, the CRC16-CCITT routine and the header-resync loop are that file's. The reader class, the auto-reconnect and the sequence-staleness rejection are new here.
- `firmware_ref/main_mcp3008.c` is `firmware/serial_test/main/main_mcp3008.c`, copied unchanged.
- `firmware_ref/main_esp32_wroom.c` is the same firmware ported to the ESP32-WROOM: UART0 through a CP2102 bridge instead of native USB CDC, with different SPI pins and clock. The calibration, smoothing and packet-building code is upstream's.

Original to this repo: `robot_interfaces/so101_serial.py`, a minimal Feetech STS3215 goal-position writer. `inhabit_teleop` targets a Unitree G1 and has no SO-101 or Feetech code. Also mine is the torque enable/disable sequencing that keeps the arm compliant until the mapper has homed, then releases it again on shutdown.

`inhabit_teleop` is a team project by Matthew Zhang and Luke Lu. The parts reused here are not solely my work, which is why the lineage is spelled out above.
