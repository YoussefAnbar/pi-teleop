"""
Minimal Feetech STS3215 writer for the SO-101 follower arm on /dev/serial0
(via the Waveshare Bus Servo Adapter in UART mode A — half-duplex, write-only).

Wire protocol (Dynamixel 1.0 compatible):
  0xFF 0xFF  ID  LEN  INSTR  PARAMS...  CHECKSUM
  LEN      = len(PARAMS) + 2
  CHECKSUM = (~(ID + LEN + INSTR + sum(PARAMS))) & 0xFF

STS3215 registers used:
  0x28 (40)  Torque Enable  (1 byte: 0 = off / compliant, 1 = on / stiff)
  0x2A (42)  Goal Position  (2 bytes LE, 0..4095 ≈ 0..360°, centre = 2048)
"""

from __future__ import annotations

import math
from typing import Iterable

import serial

INSTR_WRITE = 0x03
REG_TORQUE_ENABLE = 0x28
REG_GOAL_POSITION = 0x2A

BROADCAST_ID = 0xFE

# STS3215: 4096 counts across ~360° (2π rad)
COUNTS_PER_REV = 4096
COUNTS_PER_RAD = COUNTS_PER_REV / (2.0 * math.pi)
CENTER_COUNTS = COUNTS_PER_REV // 2  # 2048 = 0 rad


def _checksum(body: bytes) -> int:
    return (~sum(body)) & 0xFF


def _build_packet(servo_id: int, instr: int, params: bytes) -> bytes:
    length = len(params) + 2
    body = bytes([servo_id, length, instr]) + params
    return b"\xFF\xFF" + body + bytes([_checksum(body)])


def rad_to_counts(rad: float, center: int = CENTER_COUNTS) -> int:
    counts = int(round(center + rad * COUNTS_PER_RAD))
    return max(0, min(COUNTS_PER_REV - 1, counts))


class SO101Bus:
    """
    Write-only Feetech bus for the SO-101 follower arm.

    Safe default: torque is NOT enabled on construction.
    The main loop calls torque_enable_all(True) after the mapper has homed
    so the arm never jumps to an unknown position at startup.
    On shutdown the main loop calls torque_enable_all(False) so the arm
    goes compliant and can be moved by hand safely.
    """

    def __init__(
        self,
        port: str = "/dev/serial0",
        baud: int = 1_000_000,
        servo_ids: Iterable[int] = (1, 2, 3, 4, 5, 6),
        center_counts: int = CENTER_COUNTS,
    ) -> None:
        self._ser = serial.Serial(port, baudrate=baud, timeout=0.01, write_timeout=0.01)
        self._servo_ids = tuple(servo_ids)
        self._center = center_counts

    @property
    def servo_ids(self) -> tuple[int, ...]:
        return self._servo_ids

    def close(self) -> None:
        try:
            self._ser.close()
        except Exception:
            pass

    # --- torque -------------------------------------------------------

    def torque_enable(self, servo_id: int, enable: bool) -> None:
        params = bytes([REG_TORQUE_ENABLE, 0x01 if enable else 0x00])
        try:
            self._ser.write(_build_packet(servo_id, INSTR_WRITE, params))
        except serial.SerialException as exc:
            print(f"[SO101Bus] torque_enable({servo_id}) write error: {exc}", flush=True)

    def torque_enable_all(self, enable: bool) -> None:
        for sid in self._servo_ids:
            self.torque_enable(sid, enable)

    # --- goal position ------------------------------------------------

    def set_goal_position_rad(self, servo_id: int, rad: float) -> None:
        counts = rad_to_counts(rad, center=self._center)
        self.set_goal_position_counts(servo_id, counts)

    def set_goal_position_counts(self, servo_id: int, counts: int) -> None:
        lo = counts & 0xFF
        hi = (counts >> 8) & 0xFF
        params = bytes([REG_GOAL_POSITION, lo, hi])
        try:
            self._ser.write(_build_packet(servo_id, INSTR_WRITE, params))
        except serial.SerialException as exc:
            print(f"[SO101Bus] goal_position({servo_id}) write error: {exc}", flush=True)

    def write_targets(self, targets_rad: dict[int, float]) -> None:
        """Write one goal position per servo (sequential individual writes)."""
        for sid, rad in targets_rad.items():
            if sid in self._servo_ids:
                self.set_goal_position_rad(sid, rad)
