"""
Channel-to-joint mapping, extracted from programs/g1_bridge_lib_20ch.py
with every Unitree DDS / HG_LowCmd / CycloneDDS import stripped out.

Output of the mapper is a dict[servo_id] -> target_radians.  The
robot_interfaces layer decides how to translate that into wire bytes
for a specific robot (SO-101, G1, etc.).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Optional


@dataclass(frozen=True)
class JointBinding:
    """One channel -> one servo."""
    name: str
    packet_index: int          # index into the 24-float packet from the ESP32
    servo_id: int              # SO-101 / Feetech bus ID (1..6 for a 6-DoF follower)
    scale: float = 1.0         # radians per volt (or per raw unit)
    offset: float = 0.0        # radians added after scale
    min_q: Optional[float] = None       # rad — lower clamp on target
    max_q: Optional[float] = None       # rad — upper clamp on target
    average_window_size: int = 1        # moving-average window on the mapped target
    max_dq: Optional[float] = None      # rad/s — rate limit on target change
    max_angle_jump: Optional[float] = None  # rad — reject single-tick jumps > this


@dataclass(frozen=True)
class BridgeConfig:
    name: str
    joint_bindings: tuple[JointBinding, ...]
    packet_value_count: int = 24
    command_dt_sec: float = 0.01  # used by max_dq rate limit (100 Hz default)


@dataclass
class _AxisState:
    history: deque
    prev_published: Optional[float] = None


class ChannelMapper:
    """
    Apply scale / offset / clamps / rate-limit / jump-reject to each binding.

    Homes on the first packet: the channel voltage at that moment becomes
    the zero reference, so the arm will match the operator's pose at
    start-up without a calibration dance.
    """

    def __init__(self, config: BridgeConfig):
        if not config.joint_bindings:
            raise ValueError("config must define at least one binding")
        for b in config.joint_bindings:
            if b.packet_index < 0 or b.packet_index >= config.packet_value_count:
                raise ValueError(
                    f"{b.name}: packet_index {b.packet_index} out of range 0..{config.packet_value_count - 1}"
                )
            if b.average_window_size < 1:
                raise ValueError(f"{b.name}: average_window_size must be >= 1")
            if b.min_q is not None and b.max_q is not None and b.min_q > b.max_q:
                raise ValueError(f"{b.name}: min_q must be <= max_q")
            if b.max_dq is not None and b.max_dq <= 0:
                raise ValueError(f"{b.name}: max_dq must be > 0")
            if b.max_angle_jump is not None and b.max_angle_jump <= 0:
                raise ValueError(f"{b.name}: max_angle_jump must be > 0")

        self._config = config
        self._home_voltages: Optional[list[float]] = None
        self._axis_state: list[_AxisState] = [
            _AxisState(history=deque(maxlen=b.average_window_size))
            for b in config.joint_bindings
        ]

    @property
    def config(self) -> BridgeConfig:
        return self._config

    @property
    def is_homed(self) -> bool:
        return self._home_voltages is not None

    def capture_home(self, packet_values: tuple[float, ...]) -> None:
        self._home_voltages = list(packet_values)

    def apply(self, packet_values: tuple[float, ...]) -> dict[int, float]:
        """
        Translate a raw packet into {servo_id: target_radians}.

        Returns an empty dict on the very first call (it captures home).
        """
        if self._home_voltages is None:
            self.capture_home(packet_values)
            return {}

        targets: dict[int, float] = {}
        dt = self._config.command_dt_sec

        for axis, b in enumerate(self._config.joint_bindings):
            voltage = packet_values[b.packet_index]
            delta_v = voltage - self._home_voltages[b.packet_index]
            state = self._axis_state[axis]

            mapped = delta_v * b.scale + b.offset
            if b.min_q is not None:
                mapped = max(b.min_q, mapped)
            if b.max_q is not None:
                mapped = min(b.max_q, mapped)

            # stray signal rejection
            if b.max_angle_jump is not None and state.prev_published is not None:
                if abs(mapped - state.prev_published) > b.max_angle_jump:
                    mapped = state.prev_published

            # moving-average smoothing
            if not state.history:
                state.history.extend([mapped] * b.average_window_size)
            else:
                state.history.append(mapped)
            filtered = sum(state.history) / len(state.history)

            # velocity clamp
            if b.max_dq is not None and state.prev_published is not None:
                max_step = b.max_dq * dt
                diff = filtered - state.prev_published
                if diff > max_step:
                    filtered = state.prev_published + max_step
                elif diff < -max_step:
                    filtered = state.prev_published - max_step

            state.prev_published = filtered
            targets[b.servo_id] = filtered

        return targets
