# pi_teleop

Lightweight Raspberry Pi 4 runtime that maps ESP32/MCP3008 pot/encoder
input to an SO-101 follower arm over `/dev/serial0` (Waveshare Bus
Servo Adapter, UART mode A).

No PyTorch, no DDS, no MuJoCo, no GUI. Pure Python + `pyserial`.

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
├── firmware_ref/                   # ESP32 reference only — not built on Pi
└── requirements.txt                # pyserial>=3.5
```

## Install on the Pi

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

Dry-run (no servo writes; useful to confirm the ESP32 stream is valid):

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

1. Power the SO-101 with its external servo supply (not the Pi).
2. Confirm the Waveshare adapter is on UART mode A and visible at `/dev/serial0`.
3. ESP32 enumerates as `/dev/ttyACM0` (or adjust `--encoder-port`).
4. With the arm held in the desired start pose, run `--dry-run` first to confirm
   valid packets (`homed=True` and rising `valid=` counter).
5. Drop `--dry-run`. The first packet captures "home" — the arm will match the
   operator pose at that instant, then track deltas.

## Tuning

Edit `configs/so101_default.py`:
- `scale` sign → direction of each joint.
- `min_q`/`max_q` → software joint limits in radians.
- `max_dq` → rad/s rate limit (safety cap on motion speed).
- `max_angle_jump` → reject one-tick spikes from noisy pots.

## Where this came from

Derived from the reference repo at this workspace root:
- `programs/scratch/read_mcp3008_packets.py` → `encoders/mcp3008_serial.py`
- `programs/g1_bridge_lib_20ch.py` mapping math → `mapping/channel_to_joint.py`
  (all Unitree DDS / HG_LowCmd / CycloneDDS imports stripped)
- `programs/g1_POTCONFIG.py` → `configs/so101_default.py`
- `robot_interfaces/so101_serial.py` is new (no SO-101 code existed in the
  reference repo; this is a minimal Feetech STS3215 writer).
