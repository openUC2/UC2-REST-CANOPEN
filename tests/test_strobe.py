"""Strobed-sweep additions: OD constants, strobe/latch SDO sequences, SYNC frame, TPDO3 decode.

Runs without CAN hardware: `.venv/bin/python -m pytest tests/test_strobe.py`.
"""
from __future__ import annotations

import re
import struct
import sys
from pathlib import Path

import can
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from uc2canopen.od import OD, SDO_TYPES  # noqa: E402
from uc2canopen.client import Laser, Motor, State, TpdoListener, SYNC_COB_ID  # noqa: E402


class RecordingSdo:
    def __init__(self, reads=None):
        self.writes = []
        self._reads = reads or {}
        self.bus = RecordingBus()

    def _w(self, kind, n, i, s, v):
        self.writes.append((kind, n, i, s, v))
        return True

    def write_u8(self, n, i, s, v): return self._w("u8", n, i, s, v)
    def write_u16(self, n, i, s, v): return self._w("u16", n, i, s, v)
    def write_u32(self, n, i, s, v): return self._w("u32", n, i, s, v)
    def read_u8(self, n, i, s): return self._reads.get((i, s), 0)
    def read_u32(self, n, i, s): return self._reads.get((i, s), 0)


class RecordingBus:
    def __init__(self):
        self.sent = []

    def send(self, msg):
        self.sent.append(msg)


def test_constants_and_types():
    assert (OD.MOTOR_SYNC_POSITION, OD.MOTOR_SYNC_COUNT, OD.MOTOR_SYNC_LATCH_ENABLE) == (0x200C, 0x200D, 0x200E)
    assert (OD.LASER_STROBE_ENABLE, OD.LASER_STROBE_DELAY_US,
            OD.LASER_STROBE_WIDTH_US, OD.LASER_STROBE_COUNT) == (0x2107, 0x2108, 0x2109, 0x210A)
    assert SDO_TYPES[0x200C].fmt == "i" and SDO_TYPES[0x200D].fmt == "H"
    assert SDO_TYPES[0x2108].fmt == "I" and SDO_TYPES[0x2107].fmt == "B"
    assert not SDO_TYPES[0x210A].writable and SDO_TYPES[0x2109].writable


def test_constants_match_firmware_registry():
    """od.py is hand-copied from uc2-ESP's generated indices; fail loudly on drift."""
    fw = Path(__file__).resolve().parents[2] / "uc2-ESP" / "PYTHON" / "uc2_canopen" / "uc2_indices.py"
    if not fw.exists():
        pytest.skip("uc2-ESP checkout not next to this repo")
    text = fw.read_text()
    for name in ("MOTOR_SYNC_POSITION", "MOTOR_SYNC_COUNT", "MOTOR_SYNC_LATCH_ENABLE",
                 "LASER_STROBE_ENABLE", "LASER_STROBE_DELAY_US", "LASER_STROBE_WIDTH_US",
                 "LASER_STROBE_COUNT"):
        m = re.search(rf"^\s*{name}\s*=\s*(0x[0-9A-Fa-f]+)", text, re.M)
        assert m, f"{name} missing in firmware registry"
        assert int(m.group(1), 16) == getattr(OD, name), name


def test_configure_strobe_writes_delay_width_then_enable():
    sdo = RecordingSdo()
    Laser(sdo, default_node=30).configure_strobe(channel=0, enable=True, delay_us=1200, width_us=20)
    assert sdo.writes == [
        ("u32", 30, 0x2108, 1, 1200),
        ("u32", 30, 0x2109, 1, 20),
        ("u8", 30, 0x2107, 1, 1),
    ]
    sdo.writes.clear()
    Laser(sdo, default_node=30).configure_strobe(channel=0, enable=False)
    assert sdo.writes == [("u8", 30, 0x2107, 1, 0)]


def test_get_strobe_reads_back():
    sdo = RecordingSdo({(0x2107, 1): 1, (0x2108, 1): 1200, (0x2109, 1): 20, (0x210A, 1): 480})
    assert Laser(sdo, default_node=30).get_strobe(0) == {
        "enabled": True, "delay_us": 1200, "width_us": 20, "count": 480}


def test_enable_sync_latch_resets_then_enables():
    sdo = RecordingSdo()
    Motor(sdo, listener=None, default_node=11).enable_sync_latch(axis=0, enable=True)
    assert sdo.writes == [("u8", 11, 0x200E, 1, 0), ("u8", 11, 0x200E, 1, 1)]
    sdo.writes.clear()
    Motor(sdo, listener=None, default_node=11).enable_sync_latch(axis=0, enable=False)
    assert sdo.writes == [("u8", 11, 0x200E, 1, 0)]


def test_send_sync_frame():
    sdo = RecordingSdo()
    State(sdo, None).send_sync()
    (msg,) = sdo.bus.sent
    assert msg.arbitration_id == SYNC_COB_ID == 0x080
    assert not msg.is_extended_id and len(msg.data) == 0


def test_tpdo3_decode():
    listener = TpdoListener(bus=None)
    got = []
    listener.on_sync_position(lambda node, pos, count: got.append((node, pos, count)))
    listener.on_message_received(can.Message(arbitration_id=0x380 + 11,
                                             data=struct.pack("<iH", -123456, 17), is_extended_id=False))
    # short frame and other function codes are ignored
    listener.on_message_received(can.Message(arbitration_id=0x380 + 11, data=b"\x01\x02", is_extended_id=False))
    listener.on_message_received(can.Message(arbitration_id=0x480 + 11,
                                             data=struct.pack("<iH", 1, 1), is_extended_id=False))
    assert got == [(11, -123456, 17)]
