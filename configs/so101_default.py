"""
Default SO-101 follower binding: 6 MCP3008 encoder channels → 6 Feetech servo IDs.

Assumes the ESP32 firmware is built with DEBUG_RAW=0 (production mode):
  - Firmware calibrates center values at boot (arm must be at rest during boot).
  - Packet values[i] = centered deflection in [-1.5, 1.5].
  - 0.0 = encoder at its calibrated resting position.
  - ±1.5 = full deflection in each direction.

POT_SCALE converts ESP32 output units to radians on the Pi side.
  - With DEBUG_RAW=0 the firmware already normalizes: ±1.5 represents full travel.
  - For a 300° pot spanning 5.24 rad total, theoretical scale = 5.24/3.0 ≈ 1.75.
  - Start with 1.0 for first bring-up; tune each joint after verifying direction.
  - Set scale negative to invert a joint's direction.

packet_index 0..5 = MCP3008 chip-0 channels 0..5 (wire SO-101 pots to chip-0).
Chip-1/chip-2 channels (indices 8..23) are unused for SO-101 and zero in the packet.

Safety defaults (do not loosen without testing):
  max_angle_jump : rejects single-tick encoder glitches > threshold rad.
  max_dq         : velocity clamp rad/s — arm cannot step faster than this per tick.
  min_q / max_q  : hard position clamps in radians.
"""

from pi_teleop.mapping import BridgeConfig, JointBinding

POT_SCALE = 1.0  # rad per ESP32 output unit — tune per joint after bring-up

BRIDGE_CONFIG = BridgeConfig(
    name="so101_follower_default",
    packet_value_count=24,
    command_dt_sec=0.01,  # 100 Hz loop
    joint_bindings=(
        JointBinding(
            name="shoulder_pan",
            packet_index=0, servo_id=1,
            scale=POT_SCALE,
            min_q=-1.8, max_q=1.8,
            max_dq=2.5, max_angle_jump=0.5,
            average_window_size=3,
        ),
        JointBinding(
            name="shoulder_lift",
            packet_index=1, servo_id=2,
            scale=POT_SCALE,
            min_q=-1.8, max_q=1.8,
            max_dq=2.5, max_angle_jump=0.5,
            average_window_size=3,
        ),
        JointBinding(
            name="elbow_flex",
            packet_index=2, servo_id=3,
            scale=POT_SCALE,
            min_q=-1.8, max_q=1.8,
            max_dq=2.5, max_angle_jump=0.5,
            average_window_size=3,
        ),
        JointBinding(
            name="wrist_flex",
            packet_index=3, servo_id=4,
            scale=POT_SCALE,
            min_q=-1.8, max_q=1.8,
            max_dq=2.5, max_angle_jump=0.5,
            average_window_size=3,
        ),
        JointBinding(
            name="wrist_roll",
            packet_index=4, servo_id=5,
            scale=POT_SCALE,
            min_q=-3.0, max_q=3.0,
            max_dq=3.0, max_angle_jump=0.7,
            average_window_size=3,
        ),
        JointBinding(
            name="gripper",
            packet_index=5, servo_id=6,
            scale=POT_SCALE,
            min_q=-1.0, max_q=1.0,
            max_dq=3.0, max_angle_jump=0.7,
            average_window_size=3,
        ),
    ),
)
