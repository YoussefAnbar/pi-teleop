"""
Reads framed 24-float packets from the ESP32 (WROOM or S3) over serial.
On the Pi, the ESP32-WROOM enumerates as /dev/ttyUSB0 via CP2102 bridge.

Wire format (100 Hz, 102 bytes per packet):
  [0xAA 0x55]       2 bytes  header
  [uint16 LE]       2 bytes  sequence number (wraps at 65535)
  [24x float32 LE] 96 bytes  channel values
  [uint16 LE]       2 bytes  CRC16-CCITT over (sequence + values)

With DEBUG_RAW=0 (production): values are centered deflection in [-1.5, 1.5].
With DEBUG_RAW=1 (debug):      values are raw ADC voltage in [0.0, 3.3].

Auto-reconnects if the USB cable is briefly unplugged (checks every 2 s).

Dependencies: pyserial only.
"""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass
from typing import Iterator, Optional

import serial

HEADER: bytes = b"\xAA\x55"
FLOAT_COUNT: int = 24
PAYLOAD_FORMAT: str = "<H24f"
PAYLOAD_BYTES: int = struct.calcsize(PAYLOAD_FORMAT)
PACKET_BYTES: int = len(HEADER) + PAYLOAD_BYTES + 2  # header + payload + CRC16

DEFAULT_PORT: str = "/dev/ttyUSB0"
DEFAULT_BAUD: int = 115200
SERIAL_SETTLE_SEC: float = 1.0
RECONNECT_INTERVAL_SEC: float = 2.0


@dataclass
class Packet:
    sequence: int
    values: tuple[float, ...]
    received_at: float


@dataclass
class ReaderStats:
    valid: int = 0
    crc_fail: int = 0
    invalid: int = 0
    stale: int = 0
    desync_bytes: int = 0
    disconnects: int = 0


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class PacketReader:
    """
    Pull 24-float packets off a serial port with auto-reconnect on disconnect.

    Usage:
        reader = PacketReader.open("/dev/ttyUSB0", 115200)
        while True:
            pkt = reader.read_latest()   # non-blocking; None if no new packet
    """

    def __init__(
        self,
        ser: Optional[serial.Serial],
        port: str,
        baud: int,
    ) -> None:
        self._ser = ser
        self._port = port
        self._baud = baud
        self._buffer = bytearray()
        self._last_sequence: Optional[int] = None
        self._last_reconnect_attempt: float = 0.0
        self.stats = ReaderStats()

    @classmethod
    def open(cls, port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD) -> "PacketReader":
        ser = cls._open_port(port, baud)
        if ser is None:
            print(f"[PacketReader] WARNING: could not open {port} — will retry", flush=True)
        return cls(ser, port, baud)

    @staticmethod
    def _open_port(port: str, baud: int) -> Optional[serial.Serial]:
        try:
            ser = serial.Serial(port, baudrate=baud, timeout=0, write_timeout=0)
            ser.dtr = True
            ser.rts = True
            time.sleep(SERIAL_SETTLE_SEC)
            ser.reset_input_buffer()
            return ser
        except serial.SerialException:
            return None

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def read_latest(self) -> Optional[Packet]:
        """Non-blocking: drain the port and return the newest valid packet, or None."""
        self._maybe_reconnect()
        if self._ser is None:
            return None

        self._drain_into_buffer()
        latest: Optional[Packet] = None

        while len(self._buffer) >= PACKET_BYTES:
            header_index = self._buffer.find(HEADER)
            if header_index < 0:
                self.stats.desync_bytes += len(self._buffer)
                self._buffer.clear()
                break
            if header_index > 0:
                self.stats.desync_bytes += header_index
                del self._buffer[:header_index]
            if len(self._buffer) < PACKET_BYTES:
                break

            packet = self._try_parse()
            if packet is not None:
                latest = packet
            else:
                # CRC / invalid: advance one byte past the bad header and resync
                del self._buffer[:1]

        return latest

    def iter_packets(self, poll_sleep_sec: float = 0.001) -> Iterator[Packet]:
        """Blocking generator yielding every new valid packet."""
        while True:
            packet = self.read_latest()
            if packet is not None:
                yield packet
            else:
                time.sleep(poll_sleep_sec)

    # ------------------------------------------------------------------

    def _maybe_reconnect(self) -> None:
        if self._ser is not None:
            return
        now = time.monotonic()
        if now - self._last_reconnect_attempt < RECONNECT_INTERVAL_SEC:
            return
        self._last_reconnect_attempt = now
        ser = self._open_port(self._port, self._baud)
        if ser is not None:
            self._ser = ser
            self._last_sequence = None
            self._buffer.clear()
            print(f"[PacketReader] reconnected to {self._port}", flush=True)

    def _drain_into_buffer(self) -> None:
        try:
            while True:
                available = self._ser.in_waiting
                chunk = self._ser.read(available if available > 0 else 256)
                if not chunk:
                    break
                self._buffer.extend(chunk)
        except serial.SerialException:
            self._handle_disconnect()

    def _handle_disconnect(self) -> None:
        self.stats.disconnects += 1
        print(f"[PacketReader] lost {self._port} — will reconnect in "
              f"{RECONNECT_INTERVAL_SEC:.0f} s...", flush=True)
        try:
            self._ser.close()
        except Exception:
            pass
        self._ser = None
        self._buffer.clear()

    def _try_parse(self) -> Optional[Packet]:
        expected_crc = int.from_bytes(
            self._buffer[PACKET_BYTES - 2 : PACKET_BYTES], "little"
        )
        actual_crc = crc16_ccitt(bytes(self._buffer[2 : 2 + PAYLOAD_BYTES]))
        if actual_crc != expected_crc:
            self.stats.crc_fail += 1
            return None

        sequence, *values = struct.unpack_from(PAYLOAD_FORMAT, self._buffer, 2)
        del self._buffer[:PACKET_BYTES]

        if not all(math.isfinite(v) for v in values):
            self.stats.invalid += 1
            return None

        if self._last_sequence is not None:
            delta = (sequence - self._last_sequence) & 0xFFFF
            if delta == 0 or delta > 0x8000:
                self.stats.stale += 1
                return None
        self._last_sequence = sequence

        self.stats.valid += 1
        return Packet(sequence=sequence, values=tuple(values), received_at=time.monotonic())
