# pi-teleop — development journal

A Raspberry Pi teleoperation rig. An operator moves a leader arm fitted with potentiometers; an ESP32 digitises 24 channels through three MCP3008s and streams them to the Pi at 100 Hz; the Pi maps six of them onto an SO-101 follower arm over a Feetech serial bus.

**This project had no git history until 19 August 2026.** It was written as a working directory and pushed here afterwards, so there are no commits to reconstruct a timeline from. This journal is written from the source.

It sits between two other things: the V1 Unitree G1 arm (`inhabit-teleop-v1`, potentiometers and a single host) and the V2 modular joint (`Inhabit`, `Inhabit-Software`, magnetic encoders and CAN).

---

## The shape

```
leader arm potentiometers
  -> 3x MCP3008, SPI, 10-bit
  -> ESP32, ESP-IDF, FreeRTOS, 100 Hz
  -> deadzone, IIR smoothing, clamp
  -> 102-byte packet, CRC16-CCITT
  -> UART/USB to the Pi
  -> validate, home, scale, clamp, average, rate-limit
  -> Feetech STS3215 at 1 Mbaud
  -> SO-101 follower
```

Four layers with hard edges: `encoders/` knows about bytes on a wire, `mapping/` knows about angles, `robot_interfaces/` knows about Feetech registers, `configs/` holds the numbers. The mapper emits `{servo_id: radians}` and has never heard of a Feetech packet.

That seam is deliberate and it is the reason this project matters more than the arm it drives. Swapping the transport — serial for CAN — should touch one module. That is exactly what V2 did.

---

## The wire format

102 bytes at 100 Hz:

```
[0xAA 0x55][uint16 LE seq][24x float32 LE][uint16 LE CRC16-CCITT over seq+values]
```

Unpacked with `struct.unpack("<H24f", ...)`. The CRC is bit-by-bit CCITT, poly `0x1021`, init `0xFFFF`, mirroring the C implementation exactly so the two cannot disagree.

### Three checks, in order

A packet has to survive all of them:

1. **CRC** — catches corruption on the wire.
2. **Finiteness** — `math.isfinite` on all 24 floats. A NaN that reaches the mapper poisons the moving average permanently, because every subsequent output is an average involving a NaN.
3. **Sequence staleness** — `delta = (sequence - last) & 0xFFFF`, rejected if `delta == 0` or `delta > 0x8000`. A repeated or regressed sequence number means a duplicate or a reordered frame, and commanding a servo from a stale position is worse than commanding nothing.

Failures are counted, not swallowed: `valid`, `crc_fail`, `invalid`, `stale`, `desync_bytes`, `disconnects`, all printed periodically by the control loop. On a `SerialException` the reader reconnects every 2 s.

Header resync scans for `0xAA55` and advances one byte past a bad header.

### A bug in that resync

`_try_parse` deletes the packet from the buffer *before* returning `None` on the non-finite and stale paths. The caller treats `None` as a parse failure and deletes another byte:

```python
packet = self._try_parse()
if packet is not None:
    latest = packet
else:
    # CRC / invalid: advance one byte past the bad header and resync
    del self._buffer[:1]
```

So on a non-finite or stale packet it eats the first byte of the *next* frame, forcing a resync and losing one more. It self-heals through the header scan, and it needs a duplicate sequence number to trigger — most likely just after a reconnect — but it is real. The CRC-fail path is correct; it returns before deleting.

**Fix:** return a distinguishable value for "consumed but rejected" so the caller does not double-advance. Not done.

---

## The control loop

100 Hz, on an absolute deadline rather than `sleep(dt)`:

```python
next_tick += dt
sleep_for = next_tick - time.monotonic()
if sleep_for > 0:
    time.sleep(sleep_for)
else:
    next_tick = time.monotonic()
```

Sleeping for a fixed interval accumulates error — every tick drifts by however long the work took. Advancing an absolute deadline holds the average rate. Overrunning resets the deadline instead of trying to catch up, because catching up on a teleop loop means commanding a burst of stale positions.

`read_latest()` drains the buffer and returns only the newest packet. Under backlog the intermediate frames are dropped rather than replayed. Same reasoning: an operator's arm position from 200 ms ago is not information, it is a hazard.

---

## Torque sequencing

The part I would defend hardest.

Torque is **not** enabled in the constructor. It is enabled by the main loop only after the first successful mapping:

```python
if not torque_armed:
    # Enable torque only after the first homed targets are computed.
    # The arm should already be at a pose close to the operator's.
    bus.torque_enable_all(True)
    torque_armed = True
```

And disabled in a `finally`, wrapped so shutdown cannot fail:

```python
finally:
    try:
        bus.torque_enable_all(False)   # arm is now compliant
    except Exception:
        pass
```

The reason is the startup transient. The mapper homes on the first packet — the operator's current pose becomes the zero reference, so there is no calibration dance. But that means `apply()` returns `{}` on the first call, and if torque were already on, the servos would be holding whatever position they powered up at while the operator stood somewhere else. Arming on the second packet means the first command the arm ever receives is one it is already near.

On exit the arm goes limp rather than fighting whoever picks it up.

---

## Fail-closed, in the firmware

Repeated four times in the C, and it is the same principle as the status byte in V2:

> Fail-closed: if any SPI read fails, return false and leave `out[]` unchanged... so a transient bus fault never injects a false sample into the smoothing filter.

And on calibration:

> Never enter the control loop with bogus centers — that would command saturated outputs.

A smoothing filter has memory. One bad sample is not one bad output, it is a decaying error across the next several. Refusing to write is cheaper than filtering the consequence.

The WROOM firmware also silences all ESP-IDF logging on UART0, with the reason recorded rather than assumed:

> Any text on UART0 would corrupt the packet stream seen by the Pi.

---

## The mapping chain

Per axis, in order: home-relative delta → scale and offset → min/max clamp → jump reject → moving average → velocity clamp.

Actual limits, from `configs/so101_default.py`:

| joint | id | min | max | max dq | max jump | window |
|---|---|---|---|---|---|---|
| shoulder_pan | 1 | −1.8 | 1.8 | 2.5 | 0.5 | 3 |
| shoulder_lift | 2 | −1.8 | 1.8 | 2.5 | 0.5 | 3 |
| elbow_flex | 3 | −1.8 | 1.8 | 2.5 | 0.5 | 3 |
| wrist_flex | 4 | −1.8 | 1.8 | 2.5 | 0.5 | 3 |
| wrist_roll | 5 | −3.0 | 3.0 | 3.0 | 0.7 | 3 |
| gripper | 6 | −1.0 | 1.0 | 3.0 | 0.7 | 3 |

The jump rejector holds the previous value rather than passing a spike through:

```python
if abs(mapped - state.prev_published) > b.max_angle_jump:
    mapped = state.prev_published
```

A potentiometer wiper that loses contact for one sample reads garbage. Clamping that to a limit still moves the joint to the limit; holding the last good value moves it nowhere.

---

## What is not finished

**`POT_SCALE = 1.0` on every joint.** The correct value is computed in a comment directly above it and never applied:

> For a 300° pot spanning 5.24 rad total, theoretical scale = 5.24/3.0 ≈ 1.75.
> Start with 1.0 for first bring-up; tune each joint after verifying direction.

No joint has a tuned scale, an offset, or an inversion set. This is a bring-up placeholder that the project stopped at.

**The servo bus is write-only.** Documented as such — "half-duplex, write-only". No position readback, no status packets, no over-temperature or over-load polling. Nothing detects a servo that did not reach its target, and nothing notices one cooking itself. The Dynamixel-compatible protocol supports reads; this does not use them.

**Six individual writes per tick, not a SYNC_WRITE.** `BROADCAST_ID = 0xFE` is defined and unused. At 1 Mbaud six 9-byte packets is cheap, but the joints are not commanded atomically — the shoulder acts on a slightly older sample than the gripper.

**The velocity clamp uses nominal time, not measured.** `command_dt_sec = 0.01` is a constant, so a missed tick still rate-limits as though 10 ms passed, permitting a step twice as large as intended.

**Two boards, reconciled inconsistently.** `main.py` defaults to `/dev/ttyUSB0` (CP2102, WROOM) while the README bring-up says `/dev/ttyACM0` (native USB, XIAO S3). And `configs/so101_default.py` states the firmware is built with `DEBUG_RAW=0`, but `main_mcp3008.c` ships with `DEBUG_RAW 1`. Only the WROOM file is production-configured.

**No tests.** `tools/` are interactive hardware diagnostics, not automated tests.

---

## What I learned

- **Validate in layers, and count what you reject.** CRC, then finiteness, then staleness — three different failure modes needing three different checks. Six counters mean a bad run tells you which one.
- **A filter with memory turns one bad sample into many.** That is the whole argument for fail-closed reads.
- **Sequence the power.** Torque after homing, torque off on exit. Both are one line and both are the difference between a demo and an injury.
- **Absolute deadlines, not sleep intervals.** And drop stale frames rather than replaying them.
- **Write down the number you did not use.** The 1.75 scale sitting unapplied in a comment is the most useful thing in the config, because it says exactly what is left to do.

## Next

The honest next step is not a feature, it is commissioning: tune the six scales, set the inversions, and add a readback path so the follower can report that it arrived. After that, the transport swap this was structured for.
