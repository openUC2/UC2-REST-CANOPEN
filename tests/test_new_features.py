"""
Tests for the firmware-interface features added in uc2canopen 0.2.0:
galvo scanner, PID, encoder, digital/analog I/O, collision detector, plus the
extended motor / laser / led / state helpers.

These run WITHOUT hardware: each device controller is driven by a `RecordingSdo`
that captures the SDO writes/reads the controller emits, so we can assert the
exact OD index/sub/value traffic that would hit the CAN bus. This mirrors the
firmware's DeviceRouter mapping (main/src/canopen/DeviceRouter.cpp).

Run:
    pytest tests/test_new_features.py -v
    # or, without pytest installed:
    python tests/test_new_features.py

Optional live smoke test against real hardware (needs a CAN adapter + nodes):
    UC2_LIVE=1 UC2_PORT=/dev/ttyUSB0 python tests/test_new_features.py
"""

from __future__ import annotations

import os
import struct
import sys
from pathlib import Path

# Allow running straight from a source checkout without installing.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from uc2canopen.od import OD, SDO_TYPES
from uc2canopen.client import (
    Galvo, Pid, Encoder, DigitalIO, Analog, Collision, Motor, Laser, Led,
)


# ---------------------------------------------------------------------------
# Recording fake SDO client — same surface the controllers use
# ---------------------------------------------------------------------------

class RecordingSdo:
    """Stand-in for SdoClient that records writes and serves canned reads."""

    def __init__(self, reads: dict | None = None):
        # writes: list of (kind, node, index, sub, value)
        self.writes: list[tuple] = []
        # reads keyed by (index, sub) -> integer value to return
        self._reads = reads or {}
        self.bus = None  # State.send_nmt touches this; unused in these tests

    # --- write helpers ---
    def _w(self, kind, node, index, sub, value):
        self.writes.append((kind, node, index, sub, value))
        return True

    def write_u8(self, n, i, s, v):  return self._w("u8", n, i, s, v)
    def write_u16(self, n, i, s, v): return self._w("u16", n, i, s, v)
    def write_u32(self, n, i, s, v): return self._w("u32", n, i, s, v)
    def write_i32(self, n, i, s, v): return self._w("i32", n, i, s, v)

    # --- read helpers ---
    def read(self, n, i, s, size=4):
        val = self._reads.get((i, s), 0)
        return struct.pack("<h", val) if size == 2 else struct.pack("<i", val)

    def read_u8(self, n, i, s):  return self._reads.get((i, s), 0)
    def read_u16(self, n, i, s): return self._reads.get((i, s), 0)
    def read_u32(self, n, i, s): return self._reads.get((i, s), 0)
    def read_i32(self, n, i, s): return self._reads.get((i, s), 0)


def _find(writes, index, sub=None):
    """Return (kind, value) for the last write to index[/sub], or None."""
    for kind, _n, i, s, v in reversed(writes):
        if i == index and (sub is None or s == sub):
            return kind, v
    return None


# ---------------------------------------------------------------------------
# od.py — new entries are present and pack to the right widths
# ---------------------------------------------------------------------------

def test_new_od_entries_present():
    for idx in (
        OD.HOMING_STATUS, OD.LASER_DESPECKLE_PERIOD, OD.LASER_DESPECKLE_AMPLITUDE,
        OD.LED_LAYOUT_WIDTH, OD.LED_LAYOUT_HEIGHT,
        OD.COLLISION_REFERENCE, OD.COLLISION_SIGMA,
        OD.ENCODER_VELOCITY, OD.ENCODER_ZERO_OFFSET,
        OD.CPU_TEMPERATURE, OD.COMMANDED_NODE_ID,
        OD.GALVO_CAMERA_TRIGGER_MODE,
        OD.PID_SETPOINT, OD.PID_ENABLE,
    ):
        assert idx in SDO_TYPES, f"0x{idx:04X} missing from SDO_TYPES"


def test_galvo_camera_trigger_alias():
    # Renamed 0x260F to *_MODE in the firmware; the old name stays as an alias.
    assert OD.GALVO_CAMERA_TRIGGER == OD.GALVO_CAMERA_TRIGGER_MODE == 0x260F


def test_new_indices_and_widths():
    assert OD.GALVO_POINTS_DATA == 0x2610
    assert OD.BUILD_TIMESTAMP == 0x2508 and OD.MAC_ADDRESS == 0x2509
    assert SDO_TYPES[OD.CPU_TEMPERATURE].fmt == "h"   # signed 16-bit
    assert SDO_TYPES[OD.COLLISION_REFERENCE].size == 2
    assert SDO_TYPES[OD.ENCODER_VELOCITY].fmt == "i"


# ---------------------------------------------------------------------------
# Galvo
# ---------------------------------------------------------------------------

def test_galvo_goto():
    sdo = RecordingSdo()
    g = Galvo(sdo)
    g.set_position(1234, 5678, node_id=30)
    assert _find(sdo.writes, OD.GALVO_TARGET_POSITION, 1) == ("i32", 1234)
    assert _find(sdo.writes, OD.GALVO_TARGET_POSITION, 2) == ("i32", 5678)
    # command word issued last and equals GOTO
    assert sdo.writes[-1] == ("u8", 30, OD.GALVO_COMMAND_WORD, 0, Galvo.CMD_GOTO)


def test_galvo_raster_scan_maps_like_firmware():
    sdo = RecordingSdo()
    g = Galvo(sdo)
    g.raster_scan(x_min=500, x_max=3500, y_min=500, y_max=3500,
                  nx=256, ny=128, sample_period_us=3,
                  pre_samples=4, fly_samples=16, line_settle_samples=2,
                  enable_trigger=Galvo.TRIGGER_PER_PIXEL,
                  bidirectional=True, node_id=30)

    assert _find(sdo.writes, OD.GALVO_X_START) == ("i32", 500)
    assert _find(sdo.writes, OD.GALVO_Y_START) == ("i32", 500)
    assert _find(sdo.writes, OD.GALVO_N_STEPS_LINE) == ("u16", 256)
    assert _find(sdo.writes, OD.GALVO_N_STEPS_PIXEL) == ("u16", 128)
    assert _find(sdo.writes, OD.GALVO_SCAN_SPEED) == ("u32", 3)
    assert _find(sdo.writes, OD.GALVO_T_PRE_US) == ("u16", 4)
    assert _find(sdo.writes, OD.GALVO_T_POST_US) == ("u16", 16)
    assert _find(sdo.writes, OD.GALVO_CAMERA_TRIGGER_MODE) == ("u8", 1)
    # reused transport slots
    assert _find(sdo.writes, OD.GALVO_D_STEPS_LINE) == ("u16", 1)   # bidirectional
    assert _find(sdo.writes, OD.GALVO_D_STEPS_PIXEL) == ("u16", 2)  # line_settle_samples
    # step deltas computed the firmware way: (max-min)//n
    assert _find(sdo.writes, OD.GALVO_X_STEP) == ("i32", (3500 - 500) // 256)
    assert _find(sdo.writes, OD.GALVO_Y_STEP) == ("i32", (3500 - 500) // 128)
    # raster command issued last
    assert sdo.writes[-1] == ("u8", 30, OD.GALVO_COMMAND_WORD, 0, Galvo.CMD_RASTER)


def test_galvo_line_and_stop():
    sdo = RecordingSdo()
    g = Galvo(sdo)
    g.line_scan(x_min=100, x_max=900, y=2000, nx=200, node_id=30)
    assert sdo.writes[-1] == ("u8", 30, OD.GALVO_COMMAND_WORD, 0, Galvo.CMD_LINE)
    assert _find(sdo.writes, OD.GALVO_X_STEP) == ("i32", (900 - 100) // 200)
    g.stop(node_id=30)
    assert sdo.writes[-1] == ("u8", 30, OD.GALVO_COMMAND_WORD, 0, Galvo.CMD_STOP)


def test_galvo_status_decode():
    sdo = RecordingSdo(reads={(OD.GALVO_STATUS_WORD, 0): 0b0110})
    g = Galvo(sdo)
    st = g.get_status(node_id=30)
    assert st["scan_active"] and st["scan_complete"]
    assert not st["moving"] and not st["error"]


# ---------------------------------------------------------------------------
# PID / Encoder / Digital / Analog / Collision
# ---------------------------------------------------------------------------

def test_pid():
    sdo = RecordingSdo(reads={(OD.PID_ACTUAL_VALUE, 0): -42})
    p = Pid(sdo)
    p.set_setpoint(1000, node_id=50)
    p.set_gains(kp=1500, ki=10, kd=0, node_id=50)
    p.set_enabled(True, node_id=50)
    assert _find(sdo.writes, OD.PID_SETPOINT) == ("i32", 1000)
    assert _find(sdo.writes, OD.PID_KP) == ("u32", 1500)
    assert _find(sdo.writes, OD.PID_ENABLE) == ("u8", 1)
    assert p.get_actual(node_id=50) == -42


def test_encoder():
    sdo = RecordingSdo(reads={(OD.ENCODER_POSITION, 1): 12345,
                              (OD.ENCODER_VELOCITY, 1): -7})
    e = Encoder(sdo)
    assert e.get_position(axis=0, node_id=40) == 12345
    assert e.get_velocity(axis=0, node_id=40) == -7
    e.set_zero(axis=0, offset=100, node_id=40)
    assert _find(sdo.writes, OD.ENCODER_ZERO_OFFSET, 1) == ("i32", 100)


def test_digital_and_analog():
    sdo = RecordingSdo(reads={(OD.DIGITAL_INPUT_STATE, 3): 1,
                              (OD.ANALOG_INPUT_VALUE, 2): 2048})
    d = DigitalIO(sdo)
    a = Analog(sdo)
    assert d.read_input(channel=2, node_id=11) == 1     # sub = channel+1 = 3
    d.write_output(channel=0, value=1, node_id=11)
    assert _find(sdo.writes, OD.DIGITAL_OUTPUT_COMMAND, 1) == ("u8", 1)
    assert a.read_input(channel=1, node_id=11) == 2048  # sub = 2
    a.write_dac(channel=0, value=1000, node_id=11)
    assert _find(sdo.writes, OD.DAC_OUTPUT_VALUE, 1) == ("u16", 1000)


def test_collision():
    sdo = RecordingSdo(reads={(OD.COLLISION_MEAN, 0): 512,
                              (OD.COLLISION_SIGMA, 0): 7})
    c = Collision(sdo)
    c.configure(reference=500, threshold=150, sensitivity=4,
                mode=Collision.MODE_MANUAL, node_id=11)
    assert _find(sdo.writes, OD.COLLISION_MODE) == ("u8", Collision.MODE_MANUAL)
    assert _find(sdo.writes, OD.COLLISION_REFERENCE) == ("u16", 500)
    assert _find(sdo.writes, OD.COLLISION_THRESHOLD) == ("u16", 150)
    c.calibrate(node_id=11)
    assert _find(sdo.writes, OD.COLLISION_COMMAND) == ("u8", 1)
    assert c.get_mean(node_id=11) == 512
    assert c.get_sigma(node_id=11) == 7


# ---------------------------------------------------------------------------
# Extended Motor / Laser / Led helpers
# ---------------------------------------------------------------------------

class _FakeListener:
    def get_motor(self, node_id, sub=0):
        return None


def test_motor_extras():
    sdo = RecordingSdo(reads={(OD.HOMING_STATUS, 1): Motor.HOME_DONE})
    m = Motor(sdo, _FakeListener())
    assert m.get_homing_status(axis=0, node_id=11) == Motor.HOME_DONE
    m.set_soft_limits(axis=0, min_pos=-1000, max_pos=1000, node_id=11)
    assert _find(sdo.writes, OD.MOTOR_MIN_POSITION, 1) == ("i32", -1000)
    assert _find(sdo.writes, OD.MOTOR_MAX_POSITION, 1) == ("i32", 1000)
    m.configure_tmc(axis=0, microsteps=32, rms_current_ma=800,
                    stallguard_threshold=20, node_id=11)
    assert _find(sdo.writes, OD.TMC_MICROSTEPS, 1) == ("u16", 32)
    assert _find(sdo.writes, OD.TMC_RMS_CURRENT, 1) == ("u16", 800)
    m.set_hardlimit(axis=0, enabled=True, polarity=1, node_id=11)
    assert _find(sdo.writes, OD.HARDLIMIT_ENABLED, 1) == ("u8", 1)


def test_laser_extras():
    sdo = RecordingSdo(reads={(OD.LASER_MAX_VALUE, 1): 1023})
    la = Laser(sdo)
    assert la.get_max_value(channel=0, node_id=21) == 1023
    la.set_pwm_frequency(channel=0, frequency_hz=10000, node_id=21)
    la.set_despeckle(channel=0, period_us=200, amplitude=50, node_id=21)
    assert _find(sdo.writes, OD.LASER_PWM_FREQUENCY, 1) == ("u32", 10000)
    assert _find(sdo.writes, OD.LASER_DESPECKLE_PERIOD, 1) == ("u32", 200)
    assert _find(sdo.writes, OD.LASER_DESPECKLE_AMPLITUDE, 1) == ("u16", 50)


def test_led_pattern():
    sdo = RecordingSdo()
    led = Led(sdo)
    led.set_pattern(Led.PATTERN_FIRE, speed=30, node_id=20)
    assert _find(sdo.writes, OD.LED_PATTERN_SPEED) == ("u16", 30)
    assert _find(sdo.writes, OD.LED_PATTERN_ID) == ("u8", Led.PATTERN_FIRE)
    # switches into pattern mode (9) last
    assert sdo.writes[-1] == ("u8", 20, OD.LED_ARRAY_MODE, 0, Led.MODE_PATTERN)


# ---------------------------------------------------------------------------
# Optional live smoke test (real hardware)
# ---------------------------------------------------------------------------

def live_demo():
    """Exercise the new commands against real hardware. Off by default."""
    from uc2canopen import UC2Client, NODE

    port = os.environ.get("UC2_PORT")  # None -> auto-detect / SocketCAN
    uc2 = UC2Client(port=port)
    try:
        print("Scanning bus...", uc2.state.scan_nodes(timeout=2.0))

        # Galvo: centre, a small raster, then stop.
        uc2.galvo.set_position(2048, 2048, node_id=NODE.GALVO)
        uc2.galvo.raster_scan(x_min=1000, x_max=3000, y_min=1000, y_max=3000,
                              nx=64, ny=64, sample_period_us=2,
                              node_id=NODE.GALVO)
        print("Galvo status:", uc2.galvo.get_status(node_id=NODE.GALVO))
        uc2.galvo.stop(node_id=NODE.GALVO)

        # Laser despeckle + frequency.
        uc2.laser.set_pwm_frequency(0, 10000, node_id=NODE.LASER_0)
        uc2.laser.set_despeckle(0, period_us=200, amplitude=40, node_id=NODE.LASER_0)

        # LED animated pattern.
        uc2.led.set_pattern(Led.PATTERN_RAINBOW, speed=30, node_id=NODE.LED_0)

        # Encoder / PID readback (if such nodes are present).
        try:
            print("Encoder pos:", uc2.encoder.get_position(node_id=NODE.ENCODER))
        except Exception as exc:
            print("encoder read failed (node absent?):", exc)
    finally:
        uc2.close()


def _run_all():
    tests = [v for k, v in sorted(globals().items())
             if k.startswith("test_") and callable(v)]
    failures = 0
    for t in tests:
        try:
            t()
            print(f"PASS {t.__name__}")
        except AssertionError as exc:
            failures += 1
            print(f"FAIL {t.__name__}: {exc}")
    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    return failures


if __name__ == "__main__":
    if os.environ.get("UC2_LIVE") == "1":
        live_demo()
    else:
        sys.exit(1 if _run_all() else 0)
