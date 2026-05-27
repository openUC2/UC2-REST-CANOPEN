"""
Low-level CANopen SDO client for UC2.

Implements expedited SDO upload (read) and download (write) using raw
python-can frames. Does NOT require the `canopen` library — only `python-can`.

This keeps the dependency footprint minimal while still being fully
CiA 301 compliant for expedited transfers (≤ 4 bytes).
"""

from __future__ import annotations

import struct
import threading
import time
from typing import Optional

import can
from can import Listener

from .od import OD, SDO_TYPES, ODEntry


# CAN COB-ID conventions (CiA 301)
SDO_TX_BASE = 0x600   # client → server (download request / upload request)
SDO_RX_BASE = 0x580   # server → client (download response / upload response)
TPDO1_BASE  = 0x180
HB_BASE     = 0x700

# SDO command specifiers
CS_DOWNLOAD_INIT_1B = 0x2F  # expedited, 1 byte
CS_DOWNLOAD_INIT_2B = 0x2B  # expedited, 2 bytes
CS_DOWNLOAD_INIT_4B = 0x23  # expedited, 4 bytes
CS_UPLOAD_INIT      = 0x40  # initiate upload request
CS_DOWNLOAD_RSP     = 0x60  # download response
CS_UPLOAD_RSP_1B    = 0x4F  # expedited upload response, 1 byte
CS_UPLOAD_RSP_2B    = 0x4B  # expedited upload response, 2 bytes
CS_UPLOAD_RSP_4B    = 0x43  # expedited upload response, 4 bytes
CS_ABORT            = 0x80  # abort transfer

# Size → download init command byte
_DL_CMD = {1: CS_DOWNLOAD_INIT_1B, 2: CS_DOWNLOAD_INIT_2B, 4: CS_DOWNLOAD_INIT_4B}


# CiA 301 SDO abort codes — covers the ones a UC2 slave realistically returns.
# Anything not listed renders as the raw hex code in error messages.
SDO_ABORT_CODES: dict[int, str] = {
    0x05030000: "Toggle bit not alternated",
    0x05040000: "SDO protocol timed out",
    0x05040001: "Client/server command specifier not valid or unknown",
    0x05040002: "Invalid block size (block mode only)",
    0x05040003: "Invalid sequence number (block mode only)",
    0x05040004: "CRC error (block mode only)",
    0x05040005: "Out of memory",
    0x06010000: "Unsupported access to an object",
    0x06010001: "Attempt to read a write-only object",
    0x06010002: "Attempt to write a read-only object",
    0x06020000: "Object does not exist in the object dictionary",
    0x06040041: "Object cannot be mapped to the PDO",
    0x06040042: "The number and length of mapped objects would exceed PDO length",
    0x06040043: "General parameter incompatibility reason",
    0x06040047: "General internal incompatibility in the device",
    0x06060000: "Access failed due to a hardware error",
    0x06070010: "Data type does not match; length of service parameter does not match",
    0x06070012: "Data type does not match; length of service parameter too high",
    0x06070013: "Data type does not match; length of service parameter too low",
    0x06090011: "Sub-index does not exist",
    0x06090030: "Invalid value for parameter (download only)",
    0x06090031: "Value of parameter written too high (download only)",
    0x06090032: "Value of parameter written too low (download only)",
    0x06090036: "Maximum value is less than minimum value",
    0x060A0023: "Resource not available — SDO connection",
    0x08000000: "General error",
    0x08000020: "Data cannot be transferred or stored to the application",
    0x08000021: "Data cannot be transferred — local control",
    0x08000022: "Data cannot be transferred — present device state",
    0x08000023: "Object dictionary dynamic generation fails or no OD present",
    0x08000024: "No data available",
}


def describe_sdo_abort(abort_code: int) -> str:
    """Return a human-readable name for a CiA 301 SDO abort code (or hex if unknown)."""
    name = SDO_ABORT_CODES.get(abort_code)
    return f"{name} (0x{abort_code:08X})" if name else f"abort 0x{abort_code:08X}"


class SdoError(Exception):
    """Raised when an SDO transfer fails."""
    def __init__(self, message: str, abort_code: int = 0):
        self.abort_code = abort_code
        super().__init__(message)


class SdoClient(Listener):
    """
    Minimal expedited SDO client.

    Implements the python-can `Listener` interface so it can share the bus
    with other consumers (e.g. TpdoListener) via a single `can.Notifier`.
    Each incoming frame is delivered to every Listener, so there is no race
    between SDO and TPDO/heartbeat dispatch.

    Thread-safe: a lock serialises concurrent SDO requests to the same node.
    Timeout-based: if no response arrives within `timeout_s`, raises SdoError.
    """

    def __init__(self, bus: can.BusABC, timeout_s: float = 2.0,
                 settle_s: float = 0.010):
        super().__init__()
        self.bus = bus
        self.timeout_s = timeout_s
        # Small inter-transaction pause: gives the slave's CANopenNode SDO
        # server time to complete the previous transaction before the next
        # request arrives. Default raised from 5 ms to 10 ms after observing
        # CO_SDO_RT_endedWithServerTimeout (abort -9 on firmware side) during
        # multi-write sequences (e.g. motor.move = 5–6 SDO writes back-to-back).
        # Tune via UC2Client(sdo_settle_s=…) if needed.
        self.settle_s = settle_s
        self._lock = threading.Lock()
        self._pending_response: Optional[can.Message] = None
        self._response_event = threading.Event()

    # ── can.Listener interface ──
    def on_message_received(self, msg: can.Message) -> None:
        # Called by can.Notifier from its single RX thread.
        is_sdo = (msg.arbitration_id & 0x780) == SDO_RX_BASE
        print(f"[SDO listener] id=0x{msg.arbitration_id:03X} data={bytes(msg.data).hex(' ')} {'<-- SDO MATCH' if is_sdo else ''}")
        if is_sdo:
            self._pending_response = msg
            self._response_event.set()

    def on_error(self, exc: Exception) -> None:
        # python-can will log; nothing to do here.
        pass

    def stop(self) -> None:
        # Called by Notifier on shutdown.
        self._response_event.set()

    def shutdown(self):
        # Backwards-compatible no-op: the owning UC2Client now stops the
        # shared Notifier which in turn calls .stop() on each listener.
        self.stop()

    def write(self, node_id: int, index: int, sub: int,
              data: bytes) -> bool:
        """
        Expedited SDO download (write to slave).

        Args:
            node_id: target CAN node ID
            index: OD index (e.g., 0x2000)
            sub: OD sub-index (e.g., 1 for axis X)
            data: 1, 2, or 4 bytes to write (little-endian)

        Returns:
            True on success.

        Raises:
            SdoError on abort or timeout.
        """
        size = len(data)
        if size not in _DL_CMD:
            raise SdoError(f"Expedited SDO only supports 1/2/4 bytes, got {size}")

        cmd = _DL_CMD[size]
        payload = bytearray(8)
        payload[0] = cmd
        struct.pack_into("<HB", payload, 1, index, sub)
        payload[4:4+size] = data
        print(f"SDO write: node {node_id} idx 0x{index:04X}:{sub} = {data.hex()} (cmd 0x{cmd:02X})")
        with self._lock:
            self._response_event.clear()
            self._pending_response = None

            frame = can.Message(
                arbitration_id=SDO_TX_BASE + node_id,
                data=bytes(payload),
                is_extended_id=False,
            )
            print(f"Sending SDO download request to node {node_id} with frame: {frame}")
            self.bus.send(frame)

            if not self._response_event.wait(timeout=self.timeout_s):
                raise SdoError(
                    f"SDO write timeout after {self.timeout_s}s: "
                    f"node {node_id} idx 0x{index:04X}:{sub}"
                )

            resp = self._pending_response
            if resp is None:
                raise SdoError("No response received")

            # Check for abort
            if resp.data[0] == CS_ABORT:
                abort_code = struct.unpack_from("<I", resp.data, 4)[0]
                raise SdoError(
                    f"SDO write abort from node {node_id} "
                    f"idx 0x{index:04X}:{sub}: {describe_sdo_abort(abort_code)}",
                    abort_code,
                )

            # Inter-transaction settle (see __init__).
            if self.settle_s > 0:
                time.sleep(self.settle_s)

            return True

    def read(self, node_id: int, index: int, sub: int,
             size: int = 4) -> bytes:
        """
        Expedited SDO upload (read from slave).

        Args:
            node_id: target CAN node ID
            index: OD index
            sub: OD sub-index
            size: expected response size (1, 2, or 4 bytes)

        Returns:
            Raw bytes (little-endian).

        Raises:
            SdoError on abort or timeout.
        """
        payload = bytearray(8)
        payload[0] = CS_UPLOAD_INIT
        struct.pack_into("<HB", payload, 1, index, sub)

        with self._lock:
            self._response_event.clear()
            self._pending_response = None

            frame = can.Message(
                arbitration_id=SDO_TX_BASE + node_id,
                data=bytes(payload),
                is_extended_id=False,
            )
            self.bus.send(frame)

            if not self._response_event.wait(timeout=self.timeout_s):
                raise SdoError(
                    f"SDO read timeout after {self.timeout_s}s: "
                    f"node {node_id} idx 0x{index:04X}:{sub}"
                )

            resp = self._pending_response
            if resp is None:
                raise SdoError("No response received")

            if resp.data[0] == CS_ABORT:
                abort_code = struct.unpack_from("<I", resp.data, 4)[0]
                raise SdoError(
                    f"SDO read abort from node {node_id} "
                    f"idx 0x{index:04X}:{sub}: {describe_sdo_abort(abort_code)}",
                    abort_code,
                )

            data = bytes(resp.data[4:4+size])
            if self.settle_s > 0:
                time.sleep(self.settle_s)
            return data

    # ── Typed convenience helpers ──

    def write_u8(self, node_id: int, index: int, sub: int, value: int) -> bool:
        return self.write(node_id, index, sub, struct.pack("<B", value & 0xFF))

    def write_u16(self, node_id: int, index: int, sub: int, value: int) -> bool:
        return self.write(node_id, index, sub, struct.pack("<H", value & 0xFFFF))

    def write_u32(self, node_id: int, index: int, sub: int, value: int) -> bool:
        return self.write(node_id, index, sub, struct.pack("<I", value & 0xFFFFFFFF))

    def write_i32(self, node_id: int, index: int, sub: int, value: int) -> bool:
        return self.write(node_id, index, sub, struct.pack("<i", value))

    def read_u8(self, node_id: int, index: int, sub: int) -> int:
        return struct.unpack("<B", self.read(node_id, index, sub, 1))[0]

    def read_u16(self, node_id: int, index: int, sub: int) -> int:
        return struct.unpack("<H", self.read(node_id, index, sub, 2))[0]

    def read_u32(self, node_id: int, index: int, sub: int) -> int:
        return struct.unpack("<I", self.read(node_id, index, sub, 4))[0]

    def read_i32(self, node_id: int, index: int, sub: int) -> int:
        return struct.unpack("<i", self.read(node_id, index, sub, 4))[0]
