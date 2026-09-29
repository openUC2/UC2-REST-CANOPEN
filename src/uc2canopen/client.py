"""
UC2 CANopen Client — high-level Python interface for openUC2 hardware.

Mirrors the UC2-REST API shape but communicates over CAN bus via SDO instead
of JSON-over-serial. Device modules:
    uc2.motor      stepper motion, homing, TMC config, soft/hard limits
    uc2.laser      PWM intensity, frequency/resolution, despeckle
    uc2.led        uniform colour, modes, animated patterns
    uc2.galvo      goto, line/raster scan, status
    uc2.pid        generic PID control loop
    uc2.encoder    quadrature / magnetic encoder feedback
    uc2.digital    digital inputs / outputs
    uc2.analog     ADC inputs / DAC outputs
    uc2.collision  baseline-deviation collision detector
    uc2.state      firmware / system queries, node-id provisioning

Usage:
    from uc2canopen import UC2Client
    uc2 = UC2Client(port="/dev/ttyUSB0")
    uc2.motor.move(axis=1, position=1000, speed=20000)
    uc2.motor.wait_for_idle(axis=1)
    print(uc2.motor.get_position(axis=1))
    uc2.laser.set_value(channel=0, pwm=512)
    uc2.led.fill(r=255, g=0, b=0)
    uc2.galvo.raster_scan(nx=128, ny=128)
    uc2.close()
"""

from __future__ import annotations

import logging
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

import can

from .od import OD, NODE
from .sdo import SdoClient, SdoError
from .waveshare_bus import WaveshareBus, find_waveshare_port

_log = logging.getLogger(__name__)

# TPDO1 COB-ID base
TPDO1_BASE = 0x180
TPDO3_BASE = 0x380   # strobed-sweep position latch: i32 position + u16 count
SYNC_COB_ID = 0x080
HB_BASE = 0x700


# ============================================================================
# TPDO listener — background thread that caches slave state
# ============================================================================

@dataclass
class MotorState:
    position: int = 0
    running: bool = False
    homing_status: int = 0   # 0=idle, 1=homing, 2=done, 3=timeout (TPDO1 byte 5, fw >= v0.2)
    last_update: float = 0.0
    seen: bool = False


@dataclass
class NodeInfo:
    node_id: int = 0
    nmt_state: int = 0
    last_heartbeat: float = 0.0
    seen: bool = False


class TpdoListener(can.Listener):
    """Listener for TPDO1 (motor state) and heartbeat frames.

    Implements the python-can `Listener` interface so it can share the bus
    with `SdoClient` via a single `can.Notifier` — without this, both
    consumers would call `bus.recv()` independently and race for each frame
    (each frame is delivered to exactly one caller).
    """

    def __init__(self, bus: can.BusABC):
        super().__init__()
        self.bus = bus
        self.motors: dict[tuple[int, int], MotorState] = {}   # (node_id, sub) → state
        self.nodes: dict[int, NodeInfo] = {}
        self._lock = threading.Lock()
        self._callbacks: list[Callable] = []            # motor running→stopped edge
        self._update_callbacks: list[Callable] = []     # every TPDO1 frame
        self._homing_callbacks: list[Callable] = []     # homing_status transitions
        self._heartbeat_callbacks: list[Callable] = []  # every heartbeat frame
        self._sync_position_callbacks: list[Callable] = []  # every TPDO3 latch frame

    # Kept for API compatibility — no thread to start/stop now; the owning
    # UC2Client manages a shared can.Notifier instead.
    def start(self):
        pass

    def stop(self):
        # Called by Notifier on shutdown; nothing to clean up here.
        pass

    def on_error(self, exc: Exception) -> None:
        pass

    def on_motor_done(self, callback: Callable[[int, int, int], None]):
        """Register callback(node_id, sub_axis, position) for motor-done events."""
        self._callbacks.append(callback)

    def on_motor_update(self, callback: Callable[[int, int, MotorState], None]):
        """Register callback(node_id, sub_axis, MotorState) fired on every TPDO1 frame.

        This is the generic position/state stream (typically 10-50 Hz per node);
        throttle in the callback if needed.
        """
        self._update_callbacks.append(callback)

    def on_homing_changed(self, callback: Callable[[int, int, int], None]):
        """Register callback(node_id, sub_axis, homing_status) fired when the
        homing phase changes (0=idle, 1=homing, 2=done, 3=timeout)."""
        self._homing_callbacks.append(callback)

    def on_heartbeat(self, callback: Callable[[int, NodeInfo], None]):
        """Register callback(node_id, NodeInfo) fired on every heartbeat frame."""
        self._heartbeat_callbacks.append(callback)

    def on_sync_position(self, callback: Callable[[int, int, int], None]):
        """Register callback(node_id, position_steps, sync_count) for TPDO3 latch frames.

        Motor nodes send one per received SYNC while their latch (OD 0x200E) is on.
        """
        self._sync_position_callbacks.append(callback)

    def on_message_received(self, msg: can.Message) -> None:
        cob = msg.arbitration_id
        fc = cob & 0x780
        node_id = cob & 0x07F

        if fc == TPDO1_BASE and len(msg.data) >= 5:
                # TPDO1: i32 position (4 bytes) + u8 status (1 byte)
                #        + u8 homing_status (1 byte, firmware >= v0.2 — optional)
            pos = struct.unpack_from("<i", msg.data, 0)[0]
            status = msg.data[4]
            running = bool(status & 0x01)
            homing = msg.data[5] if len(msg.data) >= 6 else 0

            with self._lock:
                key = (node_id, 0)  # sub 0 for now; multi-axis slaves use sub mapping
                prev = self.motors.get(key)
                was_running = prev.running if prev and prev.seen else False
                prev_homing = prev.homing_status if prev and prev.seen else 0

                state = MotorState(
                    position=pos,
                    running=running,
                    homing_status=homing,
                    last_update=time.time(),
                    seen=True,
                )
                self.motors[key] = state

            # Callbacks fire OUTSIDE the lock: the lock is non-reentrant, so a
            # callback that reads back via get_motor()/get_node() would
            # otherwise deadlock the notifier thread.
            for cb in self._update_callbacks:
                try:
                    cb(node_id, 0, state)
                except Exception:
                    pass
            if homing != prev_homing:
                for cb in self._homing_callbacks:
                    try:
                        cb(node_id, 0, homing)
                    except Exception:
                        pass
            if was_running and not running:
                for cb in self._callbacks:
                    try:
                        cb(node_id, 0, pos)
                    except Exception:
                        pass

        elif fc == TPDO3_BASE and node_id and len(msg.data) >= 6:
            # TPDO3: i32 latched position + u16 sync count (strobed sweep)
            pos, count = struct.unpack_from("<iH", msg.data, 0)
            for cb in self._sync_position_callbacks:
                try:
                    cb(node_id, pos, count)
                except Exception:
                    pass

        elif fc == HB_BASE and len(msg.data) >= 1:
            info = NodeInfo(
                node_id=node_id,
                nmt_state=msg.data[0],
                last_heartbeat=time.time(),
                seen=True,
            )
            with self._lock:
                self.nodes[node_id] = info
            for cb in self._heartbeat_callbacks:
                try:
                    cb(node_id, info)
                except Exception:
                    pass

    def get_motor(self, node_id: int, sub: int = 0) -> Optional[MotorState]:
        with self._lock:
            return self.motors.get((node_id, sub))

    def get_node(self, node_id: int) -> Optional[NodeInfo]:
        with self._lock:
            return self.nodes.get(node_id)

    def scan(self, timeout: float = 3.0) -> list[int]:
        """Return node IDs seen via heartbeat or TPDO within timeout."""
        found: set[int] = set()
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                for nid, info in self.nodes.items():
                    if info.seen:
                        found.add(nid)
                for (nid, _), ms in self.motors.items():
                    if ms.seen:
                        found.add(nid)
            time.sleep(0.1)
        return sorted(found)


# ============================================================================
# Motor controller
# ============================================================================

class Motor:
    """
    High-level motor control matching UC2-REST's motor API.

    UC2 slave's motor command protocol:
        1. Write MOTOR_TARGET_POSITION[sub] = position
        2. Write MOTOR_SPEED[sub] = speed
        3. Write MOTOR_ACCELERATION[sub] = accel (if >0)
        4. Write MOTOR_IS_ABSOLUTE[sub] = 0 or 1
        5. Write MOTOR_IS_FOREVER[sub] = 0 or 1
        6. Write MOTOR_COMMAND_WORD = bitmask (bit N = start axis N, bit N+4 = stop)
    """

    def __init__(self, sdo: SdoClient, listener: TpdoListener,
                 default_node: int = NODE.MOT_X):
        self._sdo = sdo
        self._listener = listener
        self.default_node = default_node

    def enable_sync_latch(self, axis: int = 0, enable: bool = True,
                          node_id: Optional[int] = None) -> bool:
        """Latch this node's position on every SYNC and send it in TPDO3 (0x380 + node).

        Enabling writes 0 first so the node restarts its count at 1 for the next
        SYNC (it polls the object every millisecond).
        """
        nid = node_id or self.default_node
        sub = axis + 1
        if enable:
            self._sdo.write_u8(nid, OD.MOTOR_SYNC_LATCH_ENABLE, sub, 0)
            time.sleep(0.01)
        return bool(self._sdo.write_u8(nid, OD.MOTOR_SYNC_LATCH_ENABLE, sub, 1 if enable else 0))

    def move(self, axis: int = 0, position: int = 0, speed: int = 20000,
             acceleration: int = 0, is_absolute: bool = False,
             is_forever: bool = False, node_id: Optional[int] = None):
        """
        Start a motor move.

        Args:
            axis: axis index on the slave (0-3; typically 0 for single-motor slaves)
            position: target position in steps (relative or absolute)
            speed: max speed in steps/s
            acceleration: steps/s² (0 = use slave default)
            is_absolute: False = relative move, True = absolute
            is_forever: True = continuous move (ignores position)
            node_id: CAN node ID (default: self.default_node)
        """
        nid = node_id or self.default_node
        sub = axis + 1  # OD sub-indices are 1-based

        _log.debug("Motor move: node %d axis %d -> pos %d at %d steps/s "
                   "(sub=%d accel=%d absolute=%s forever=%s)",
                   nid, axis, position, speed, sub, acceleration, is_absolute, is_forever)
        self._sdo.write_i32(nid, OD.MOTOR_TARGET_POSITION, sub, position)
        _log.debug("Setting speed %d steps/s for node %d axis %d (OD sub %d)",
                   speed, nid, axis, sub)
        self._sdo.write_u32(nid, OD.MOTOR_SPEED, sub, speed)
        
        if acceleration > 0:
            self._sdo.write_u32(nid, OD.MOTOR_ACCELERATION, sub, acceleration)
        self._sdo.write_u8(nid, OD.MOTOR_IS_ABSOLUTE, sub, 1 if is_absolute else 0)
        self._sdo.write_u8(nid, OD.MOTOR_IS_FOREVER, sub, 1 if is_forever else 0)

        # Trigger: set bit `axis` in the command word
        cmd_word = 1 << axis
        self._sdo.write_u8(nid, OD.MOTOR_COMMAND_WORD, 0, cmd_word)

    def stop(self, axis: int = 0, node_id: Optional[int] = None):
        """Stop a motor axis."""
        nid = node_id or self.default_node
        cmd_word = 1 << (axis + 4)  # bits 4-7 = stop axis 0-3
        self._sdo.write_u8(nid, OD.MOTOR_COMMAND_WORD, 0, cmd_word)

    def get_position(self, axis: int = 0, node_id: Optional[int] = None) -> int:
        """Read motor position. Prefers TPDO cache; falls back to SDO read."""
        nid = node_id or self.default_node
        state = self._listener.get_motor(nid, 0)
        if state and state.seen and (time.time() - state.last_update) < 2.0:
            return state.position
        # Fallback: SDO read
        return self._sdo.read_i32(nid, OD.MOTOR_ACTUAL_POSITION, axis + 1)

    def is_running(self, axis: int = 0, node_id: Optional[int] = None) -> bool:
        """Check if motor is running. Uses TPDO cache if available."""
        nid = node_id or self.default_node
        state = self._listener.get_motor(nid, 0)
        if state and state.seen and (time.time() - state.last_update) < 2.0:
            return state.running
        # Fallback: SDO read
        status = self._sdo.read_u8(nid, OD.MOTOR_STATUS_WORD, axis + 1)
        return bool(status & 0x01)

    def wait_for_idle(self, axis: int = 0, node_id: Optional[int] = None,
                      timeout: float = 30.0) -> bool:
        """Block until motor stops or timeout. Returns True if idle."""
        nid = node_id or self.default_node
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self.is_running(axis, nid):
                return True
            time.sleep(0.05)
        return False

    def home(self, axis: int = 0, speed: int = 15000, direction: int = -1,
             timeout_ms: int = 20000, node_id: Optional[int] = None):
        """Start homing for an axis."""
        nid = node_id or self.default_node
        sub = axis + 1
        self._sdo.write_u32(nid, OD.HOMING_SPEED, sub, speed)
        self._sdo.write_u8(nid, OD.HOMING_DIRECTION, sub, direction & 0xFF)
        self._sdo.write_u32(nid, OD.HOMING_TIMEOUT, sub, timeout_ms)
        self._sdo.write_u8(nid, OD.HOMING_COMMAND, sub, 1)

    def set_enabled(self, axis: int = 0, enabled: bool = True,
                    node_id: Optional[int] = None):
        """Enable/disable motor driver."""
        nid = node_id or self.default_node
        self._sdo.write_u8(nid, OD.MOTOR_ENABLE, axis + 1, 1 if enabled else 0)

    # Homing status codes reported via HOMING_STATUS (0x2016) / TPDO1 byte 5.
    HOME_IDLE = 0
    HOME_HOMING = 1
    HOME_DONE = 2
    HOME_TIMEOUT = 3

    def get_homing_status(self, axis: int = 0, node_id: Optional[int] = None) -> int:
        """Return homing phase for an axis (0=idle, 1=homing, 2=done, 3=timeout).

        Prefers the TPDO cache (axis 0) if fresh, else falls back to an SDO read.
        """
        nid = node_id or self.default_node
        if axis == 0:
            state = self._listener.get_motor(nid, 0)
            if state and state.seen and (time.time() - state.last_update) < 2.0:
                return state.homing_status
        return self._sdo.read_u8(nid, OD.HOMING_STATUS, axis + 1)

    def wait_for_homed(self, axis: int = 0, node_id: Optional[int] = None,
                       timeout: float = 30.0) -> bool:
        """Block until homing completes. Returns True if HOME_DONE, False on timeout."""
        nid = node_id or self.default_node
        deadline = time.time() + timeout
        while time.time() < deadline:
            st = self.get_homing_status(axis, nid)
            if st == self.HOME_DONE:
                return True
            if st == self.HOME_TIMEOUT:
                return False
            time.sleep(0.05)
        return False

    def set_soft_limits(self, axis: int = 0, min_pos: Optional[int] = None,
                        max_pos: Optional[int] = None, node_id: Optional[int] = None):
        """Set soft position limits (steps). The slave clips targets to this range."""
        nid = node_id or self.default_node
        sub = axis + 1
        if min_pos is not None:
            self._sdo.write_i32(nid, OD.MOTOR_MIN_POSITION, sub, min_pos)
        if max_pos is not None:
            self._sdo.write_i32(nid, OD.MOTOR_MAX_POSITION, sub, max_pos)

    def set_acceleration(self, axis: int = 0, acceleration: int = 100000,
                         node_id: Optional[int] = None):
        """Set the acceleration profile (steps/s²) used for subsequent moves."""
        nid = node_id or self.default_node
        self._sdo.write_u32(nid, OD.MOTOR_ACCELERATION, axis + 1, acceleration)

    def configure_tmc(self, axis: int = 0, microsteps: Optional[int] = None,
                      rms_current_ma: Optional[int] = None,
                      stallguard_threshold: Optional[int] = None,
                      node_id: Optional[int] = None):
        """Configure the TMC2209 driver for an axis (SDO-only, set-once)."""
        nid = node_id or self.default_node
        sub = axis + 1
        if microsteps is not None:
            self._sdo.write_u16(nid, OD.TMC_MICROSTEPS, sub, microsteps)
        if rms_current_ma is not None:
            self._sdo.write_u16(nid, OD.TMC_RMS_CURRENT, sub, rms_current_ma)
        if stallguard_threshold is not None:
            self._sdo.write_u8(nid, OD.TMC_STALLGUARD_THRESHOLD, sub, stallguard_threshold)

    def set_hardlimit(self, axis: int = 0, enabled: bool = True,
                      polarity: int = 0, node_id: Optional[int] = None):
        """Enable/disable the hard-limit endstop for an axis and set its polarity."""
        nid = node_id or self.default_node
        sub = axis + 1
        self._sdo.write_u8(nid, OD.HARDLIMIT_ENABLED, sub, 1 if enabled else 0)
        self._sdo.write_u8(nid, OD.HARDLIMIT_POLARITY, sub, polarity & 0xFF)

    def clear_hardlimit(self, axis: int = 0, node_id: Optional[int] = None):
        """Clear a latched hard-limit trip on an axis."""
        nid = node_id or self.default_node
        self._sdo.write_u8(nid, OD.HARDLIMIT_COMMAND, axis + 1, 1)


# ============================================================================
# Laser controller
# ============================================================================

class Laser:
    """Laser control — matches UC2-REST's laser API."""

    def __init__(self, sdo: SdoClient, default_node: int = NODE.LASER_0):
        self._sdo = sdo
        self.default_node = default_node

    def set_value(self, channel: int = 0, pwm: int = 0,
                  node_id: Optional[int] = None):
        """Set laser PWM value (0 = off, up to laser_max_value)."""
        nid = node_id or self.default_node
        self._sdo.write_u16(nid, OD.LASER_PWM_VALUE, channel + 1, pwm)

    def get_value(self, channel: int = 0, node_id: Optional[int] = None) -> int:
        nid = node_id or self.default_node
        return self._sdo.read_u16(nid, OD.LASER_PWM_VALUE, channel + 1)

    def off(self, channel: int = 0, node_id: Optional[int] = None):
        self.set_value(channel, 0, node_id)

    def set_all(self, pwm: int, num_channels: int = 3,
                node_id: Optional[int] = None):
        """Set all laser channels to the same PWM value."""
        for ch in range(num_channels):
            self.set_value(ch, pwm, node_id)

    def get_max_value(self, channel: int = 0, node_id: Optional[int] = None) -> int:
        """Read the full-scale PWM value (depends on PWM resolution)."""
        nid = node_id or self.default_node
        return self._sdo.read_u16(nid, OD.LASER_MAX_VALUE, channel + 1)

    def set_pwm_frequency(self, channel: int = 0, frequency_hz: int = 5000,
                          node_id: Optional[int] = None):
        """Set the PWM carrier frequency (Hz). freq * 2^resolution must be <= APB clock."""
        nid = node_id or self.default_node
        self._sdo.write_u32(nid, OD.LASER_PWM_FREQUENCY, channel + 1, frequency_hz)

    def set_pwm_resolution(self, channel: int = 0, bits: int = 10,
                           node_id: Optional[int] = None):
        """Set the PWM resolution in bits (full scale becomes 2^bits - 1)."""
        nid = node_id or self.default_node
        self._sdo.write_u8(nid, OD.LASER_PWM_RESOLUTION, channel + 1, bits & 0xFF)

    def configure_strobe(self, channel: int = 0, enable: bool = True, delay_us: int = 0,
                         width_us: int = 20, node_id: Optional[int] = None) -> bool:
        """Strobe this node channel: one flash per received SYNC, delay_us after it.

        Delay and width are written before enable so the node never flashes with
        stale values. The node applies the request from its main loop and writes
        the outcome back (enable 0 when refused, clamped delay/width): read it with
        get_strobe() a few tens of milliseconds later.
        """
        nid = node_id or self.default_node
        sub = channel + 1
        if enable:
            self._sdo.write_u32(nid, OD.LASER_STROBE_DELAY_US, sub, int(delay_us))
            self._sdo.write_u32(nid, OD.LASER_STROBE_WIDTH_US, sub, int(width_us))
        return bool(self._sdo.write_u8(nid, OD.LASER_STROBE_ENABLE, sub, 1 if enable else 0))

    def get_strobe(self, channel: int = 0, node_id: Optional[int] = None) -> dict:
        """Read back {'enabled', 'delay_us', 'width_us', 'count'} of a strobe channel."""
        nid = node_id or self.default_node
        sub = channel + 1
        return {
            "enabled": bool(self._sdo.read_u8(nid, OD.LASER_STROBE_ENABLE, sub)),
            "delay_us": self._sdo.read_u32(nid, OD.LASER_STROBE_DELAY_US, sub),
            "width_us": self._sdo.read_u32(nid, OD.LASER_STROBE_WIDTH_US, sub),
            "count": self._sdo.read_u32(nid, OD.LASER_STROBE_COUNT, sub),
        }

    def set_despeckle(self, channel: int = 0, period_us: int = 0, amplitude: int = 0,
                      node_id: Optional[int] = None):
        """Configure laser despeckle modulation. period_us=0 disables it."""
        nid = node_id or self.default_node
        sub = channel + 1
        self._sdo.write_u32(nid, OD.LASER_DESPECKLE_PERIOD, sub, period_us)
        self._sdo.write_u16(nid, OD.LASER_DESPECKLE_AMPLITUDE, sub, amplitude)

    def get_safety_state(self, node_id: Optional[int] = None) -> int:
        """Read the laser safety bitfield (bit0=interlock_ok, bit1=thermal_ok, bit2=enabled)."""
        nid = node_id or self.default_node
        return self._sdo.read_u8(nid, OD.LASER_SAFETY_STATE, 0)


# ============================================================================
# LED controller
# ============================================================================

class Led:
    """LED matrix control — matches UC2-REST's led API."""

    def __init__(self, sdo: SdoClient, default_node: int = NODE.LED_0):
        self._sdo = sdo
        self.default_node = default_node

    def fill(self, r: int = 0, g: int = 0, b: int = 0,
             node_id: Optional[int] = None):
        """Fill all LEDs with a uniform color."""
        nid = node_id or self.default_node
        colour = ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF)
        self._sdo.write_u32(nid, OD.LED_UNIFORM_COLOUR, 0, colour)
        self._sdo.write_u8(nid, OD.LED_ARRAY_MODE, 0, 1)  # mode 1 = fill

    def off(self, node_id: Optional[int] = None):
        """Turn all LEDs off."""
        nid = node_id or self.default_node
        self._sdo.write_u8(nid, OD.LED_ARRAY_MODE, 0, 0)

    def set_brightness(self, brightness: int = 128, node_id: Optional[int] = None):
        nid = node_id or self.default_node
        self._sdo.write_u8(nid, OD.LED_BRIGHTNESS, 0, brightness & 0xFF)

    # LED_ARRAY_MODE values (0x2200)
    MODE_OFF = 0
    MODE_ON = 1
    MODE_LEFT = 2
    MODE_RIGHT = 3
    MODE_TOP = 4
    MODE_BOTTOM = 5
    MODE_INNER_RING = 6
    MODE_OUTER_RING = 7
    MODE_CUSTOM = 8
    MODE_PATTERN = 9

    def set_mode(self, mode: int, node_id: Optional[int] = None):
        """Set the LED array mode (see Led.MODE_* constants)."""
        nid = node_id or self.default_node
        self._sdo.write_u8(nid, OD.LED_ARRAY_MODE, 0, mode & 0xFF)

    # LED_PATTERN_ID values (0x2220)
    PATTERN_NONE = 0
    PATTERN_RAINBOW = 1
    PATTERN_BREATHE = 2
    PATTERN_CHASE = 3
    PATTERN_FIRE = 4
    PATTERN_SPARKLE = 5
    PATTERN_HEATMAP = 6
    PATTERN_SPIRAL = 7

    def set_pattern(self, pattern_id: int = PATTERN_RAINBOW, speed: Optional[int] = None,
                    node_id: Optional[int] = None):
        """Start a built-in animated pattern (see Led.PATTERN_* constants).

        Sets the pattern id, optional animation speed (fps) and switches the
        array into pattern mode (mode 9).
        """
        nid = node_id or self.default_node
        if speed is not None:
            self._sdo.write_u16(nid, OD.LED_PATTERN_SPEED, 0, speed & 0xFFFF)
        self._sdo.write_u8(nid, OD.LED_PATTERN_ID, 0, pattern_id & 0xFF)
        self._sdo.write_u8(nid, OD.LED_ARRAY_MODE, 0, self.MODE_PATTERN)

    def get_pixel_count(self, node_id: Optional[int] = None) -> int:
        """Read the number of physical LEDs on this node."""
        nid = node_id or self.default_node
        return self._sdo.read_u16(nid, OD.LED_PIXEL_COUNT, 0)


# ============================================================================
# Galvo / scanner controller
# ============================================================================

class Galvo:
    """
    Galvo mirror scanner control — matches UC2-REST's /galvo_act API.

    The slave's galvo command word (OD 0x2602) is a doorbell: write a nonzero
    code, the slave executes it and clears the word back to CMD_IDLE. Stop has
    its own nonzero code (CMD_STOP) so the self-clear is never mistaken for a
    stop.

    Positions are in DAC counts (0..4095 for a 12-bit MCP4822).
    """

    # Command word values (OD 0x2602) — shared master/slave protocol.
    CMD_IDLE = 0
    CMD_GOTO = 1
    CMD_LINE = 2
    CMD_RASTER = 3
    CMD_STOP = 4
    CMD_ESTOP = 5

    # Camera trigger modes (OD 0x260F)
    TRIGGER_OFF = 0
    TRIGGER_PER_PIXEL = 1
    TRIGGER_PER_LINE = 2
    TRIGGER_PER_FRAME = 3

    def __init__(self, sdo: SdoClient, default_node: int = NODE.GALVO):
        self._sdo = sdo
        self.default_node = default_node

    def set_position(self, x: int, y: int, node_id: Optional[int] = None):
        """Move the galvo to an absolute XY position (DAC counts) and issue GOTO."""
        nid = node_id or self.default_node
        self._sdo.write_i32(nid, OD.GALVO_TARGET_POSITION, 1, x)
        self._sdo.write_i32(nid, OD.GALVO_TARGET_POSITION, 2, y)
        self._sdo.write_u8(nid, OD.GALVO_COMMAND_WORD, 0, self.CMD_GOTO)

    # Alias matching UC2-REST naming.
    goto = set_position

    def get_position(self, node_id: Optional[int] = None) -> tuple[int, int]:
        """Read the current galvo XY position (DAC counts)."""
        nid = node_id or self.default_node
        x = self._sdo.read_i32(nid, OD.GALVO_ACTUAL_POSITION, 1)
        y = self._sdo.read_i32(nid, OD.GALVO_ACTUAL_POSITION, 2)
        return x, y

    def raster_scan(self, x_min: int = 500, x_max: int = 3500,
                    y_min: int = 500, y_max: int = 3500,
                    nx: int = 256, ny: int = 256,
                    sample_period_us: int = 1,
                    pre_samples: int = 4, fly_samples: int = 16,
                    line_settle_samples: int = 0,
                    enable_trigger: int = TRIGGER_PER_PIXEL,
                    bidirectional: bool = False,
                    node_id: Optional[int] = None):
        """
        Configure and start a raster (full XY) scan.

        Mirrors the firmware DeviceRouter::handleGalvoAct "config" path: writes
        all scan parameters via SDO, computes x_step / y_step from the ranges and
        step counts, then issues CMD_RASTER.

        Args:
            x_min/x_max/y_min/y_max: scan bounding box in DAC counts (0..4095)
            nx/ny: number of steps along X (line) and Y (pixel/frame)
            sample_period_us: dwell time per point (maps to GALVO_SCAN_SPEED)
            pre_samples/fly_samples: blanking / flyback samples (t_pre/t_post)
            line_settle_samples: settling samples after flyback
            enable_trigger: camera trigger mode (see Galvo.TRIGGER_* constants)
            bidirectional: True = scan alternate lines in reverse
        """
        nid = node_id or self.default_node
        self._sdo.write_i32(nid, OD.GALVO_X_START, 0, x_min)
        self._sdo.write_i32(nid, OD.GALVO_Y_START, 0, y_min)
        self._sdo.write_u16(nid, OD.GALVO_N_STEPS_LINE, 0, nx)
        self._sdo.write_u16(nid, OD.GALVO_N_STEPS_PIXEL, 0, ny)
        self._sdo.write_u32(nid, OD.GALVO_SCAN_SPEED, 0, sample_period_us)
        self._sdo.write_u16(nid, OD.GALVO_T_PRE_US, 0, pre_samples)
        self._sdo.write_u16(nid, OD.GALVO_T_POST_US, 0, fly_samples)
        self._sdo.write_u8(nid, OD.GALVO_CAMERA_TRIGGER_MODE, 0, enable_trigger)
        # These two galvo slots are REUSED as transport containers by the firmware:
        #   GALVO_D_STEPS_LINE  (0x2607) -> bidirectional (0/1)
        #   GALVO_D_STEPS_PIXEL (0x2608) -> line_settle_samples
        self._sdo.write_u16(nid, OD.GALVO_D_STEPS_LINE, 0, 1 if bidirectional else 0)
        self._sdo.write_u16(nid, OD.GALVO_D_STEPS_PIXEL, 0, line_settle_samples)
        # Compute per-step deltas the same way the firmware does.
        if nx > 0:
            self._sdo.write_i32(nid, OD.GALVO_X_STEP, 0, (x_max - x_min) // nx)
        if ny > 0:
            self._sdo.write_i32(nid, OD.GALVO_Y_STEP, 0, (y_max - y_min) // ny)
        self._sdo.write_u8(nid, OD.GALVO_COMMAND_WORD, 0, self.CMD_RASTER)

    def line_scan(self, x_min: int = 500, x_max: int = 3500,
                  y: int = 2000, nx: int = 256,
                  sample_period_us: int = 1,
                  enable_trigger: int = TRIGGER_PER_PIXEL,
                  node_id: Optional[int] = None):
        """Configure and start a single-line (X sweep) scan at a fixed Y."""
        nid = node_id or self.default_node
        self._sdo.write_i32(nid, OD.GALVO_X_START, 0, x_min)
        self._sdo.write_i32(nid, OD.GALVO_Y_START, 0, y)
        self._sdo.write_u16(nid, OD.GALVO_N_STEPS_LINE, 0, nx)
        self._sdo.write_u32(nid, OD.GALVO_SCAN_SPEED, 0, sample_period_us)
        self._sdo.write_u8(nid, OD.GALVO_CAMERA_TRIGGER_MODE, 0, enable_trigger)
        if nx > 0:
            self._sdo.write_i32(nid, OD.GALVO_X_STEP, 0, (x_max - x_min) // nx)
        self._sdo.write_u8(nid, OD.GALVO_COMMAND_WORD, 0, self.CMD_LINE)

    def stop(self, node_id: Optional[int] = None):
        """Stop any active scan / motion."""
        nid = node_id or self.default_node
        self._sdo.write_u8(nid, OD.GALVO_COMMAND_WORD, 0, self.CMD_STOP)

    def estop(self, node_id: Optional[int] = None):
        """Emergency stop."""
        nid = node_id or self.default_node
        self._sdo.write_u8(nid, OD.GALVO_COMMAND_WORD, 0, self.CMD_ESTOP)

    def get_status(self, node_id: Optional[int] = None) -> dict:
        """Read the galvo status word and decode it into named flags."""
        nid = node_id or self.default_node
        s = self._sdo.read_u8(nid, OD.GALVO_STATUS_WORD, 0)
        return {
            "raw": s,
            "moving": bool(s & 0x01),
            "scan_active": bool(s & 0x02),
            "scan_complete": bool(s & 0x04),
            "error": bool(s & 0x08),
        }


# ============================================================================
# PID controller
# ============================================================================

class Pid:
    """Generic PID control loop (e.g. focus stabilization) — OD 0x2700."""

    def __init__(self, sdo: SdoClient, default_node: int = NODE.PID):
        self._sdo = sdo
        self.default_node = default_node

    def set_setpoint(self, setpoint: int, node_id: Optional[int] = None):
        nid = node_id or self.default_node
        self._sdo.write_i32(nid, OD.PID_SETPOINT, 0, setpoint)

    def get_actual(self, node_id: Optional[int] = None) -> int:
        """Read the current process value."""
        nid = node_id or self.default_node
        return self._sdo.read_i32(nid, OD.PID_ACTUAL_VALUE, 0)

    def set_gains(self, kp: Optional[int] = None, ki: Optional[int] = None,
                  kd: Optional[int] = None, node_id: Optional[int] = None):
        """Set PID gains (x1000 fixed point). Only provided gains are written."""
        nid = node_id or self.default_node
        if kp is not None:
            self._sdo.write_u32(nid, OD.PID_KP, 0, kp)
        if ki is not None:
            self._sdo.write_u32(nid, OD.PID_KI, 0, ki)
        if kd is not None:
            self._sdo.write_u32(nid, OD.PID_KD, 0, kd)

    def set_enabled(self, enabled: bool = True, node_id: Optional[int] = None):
        nid = node_id or self.default_node
        self._sdo.write_u8(nid, OD.PID_ENABLE, 0, 1 if enabled else 0)


# ============================================================================
# Encoder feedback
# ============================================================================

class Encoder:
    """Quadrature / magnetic (AS5600) encoder feedback — OD 0x2340."""

    def __init__(self, sdo: SdoClient, default_node: int = NODE.ENCODER):
        self._sdo = sdo
        self.default_node = default_node

    def get_position(self, axis: int = 0, node_id: Optional[int] = None) -> int:
        """Read encoder position in counts."""
        nid = node_id or self.default_node
        return self._sdo.read_i32(nid, OD.ENCODER_POSITION, axis + 1)

    def get_velocity(self, axis: int = 0, node_id: Optional[int] = None) -> int:
        """Read encoder velocity in counts/s."""
        nid = node_id or self.default_node
        return self._sdo.read_i32(nid, OD.ENCODER_VELOCITY, axis + 1)

    def set_zero(self, axis: int = 0, offset: int = 0, node_id: Optional[int] = None):
        """Set the zero offset applied to raw encoder counts."""
        nid = node_id or self.default_node
        self._sdo.write_i32(nid, OD.ENCODER_ZERO_OFFSET, axis + 1, offset)


# ============================================================================
# Digital I/O
# ============================================================================

class DigitalIO:
    """Digital inputs / outputs (endstops, triggers, GPIOs) — OD 0x2300."""

    def __init__(self, sdo: SdoClient, default_node: int = NODE.MOT_X):
        self._sdo = sdo
        self.default_node = default_node

    def read_input(self, channel: int = 0, node_id: Optional[int] = None) -> int:
        """Read one digital input channel (0/1)."""
        nid = node_id or self.default_node
        return self._sdo.read_u8(nid, OD.DIGITAL_INPUT_STATE, channel + 1)

    def write_output(self, channel: int = 0, value: int = 0,
                     node_id: Optional[int] = None):
        """Set one digital output channel (0/1)."""
        nid = node_id or self.default_node
        self._sdo.write_u8(nid, OD.DIGITAL_OUTPUT_COMMAND, channel + 1, 1 if value else 0)


# ============================================================================
# Analog I/O (ADC in / DAC out)
# ============================================================================

class Analog:
    """Analog inputs (ADC) and outputs (DAC) — OD 0x2310 / 0x2320."""

    def __init__(self, sdo: SdoClient, default_node: int = NODE.MOT_X):
        self._sdo = sdo
        self.default_node = default_node

    def read_input(self, channel: int = 0, node_id: Optional[int] = None) -> int:
        """Read a raw ADC input value (counts)."""
        nid = node_id or self.default_node
        return self._sdo.read_u16(nid, OD.ANALOG_INPUT_VALUE, channel + 1)

    def read_filtered(self, channel: int = 0, node_id: Optional[int] = None) -> int:
        """Read an EWMA-filtered ADC input value (counts)."""
        nid = node_id or self.default_node
        return self._sdo.read_u16(nid, OD.ANALOG_INPUT_FILTERED, channel + 1)

    def write_dac(self, channel: int = 0, value: int = 0,
                  node_id: Optional[int] = None):
        """Write a DAC output value (counts)."""
        nid = node_id or self.default_node
        self._sdo.write_u16(nid, OD.DAC_OUTPUT_VALUE, channel + 1, value)


# ============================================================================
# Collision detector (GPIO slave)
# ============================================================================

class Collision:
    """
    Baseline-deviation collision detector on the GPIO slave — OD 0x2330.

    Two modes: AUTO (adaptive baseline + robust sigma z-score, parameter-free)
    and MANUAL (fixed reference +/- threshold). The trip event rides TPDO2; the
    rolling mean and sigma are polled via SDO on demand.
    """

    MODE_AUTO = 0
    MODE_MANUAL = 1

    def __init__(self, sdo: SdoClient, default_node: int = NODE.MOT_X):
        self._sdo = sdo
        self.default_node = default_node

    def configure(self, reference: Optional[int] = None,
                  threshold: Optional[int] = None,
                  sensitivity: Optional[int] = None,
                  mode: Optional[int] = None,
                  node_id: Optional[int] = None):
        """Configure the (MANUAL-mode) detector. Only provided fields are written."""
        nid = node_id or self.default_node
        if mode is not None:
            self._sdo.write_u8(nid, OD.COLLISION_MODE, 0, mode)
        if reference is not None:
            self._sdo.write_u16(nid, OD.COLLISION_REFERENCE, 0, reference)
        if threshold is not None:
            self._sdo.write_u16(nid, OD.COLLISION_THRESHOLD, 0, threshold)
        if sensitivity is not None:
            self._sdo.write_u8(nid, OD.COLLISION_SENSITIVITY, 0, sensitivity)

    def calibrate(self, node_id: Optional[int] = None):
        """Calibrate: set reference := current rolling mean and persist it."""
        nid = node_id or self.default_node
        self._sdo.write_u8(nid, OD.COLLISION_COMMAND, 0, 1)

    def get_mean(self, node_id: Optional[int] = None) -> int:
        """Poll the rolling mean / adaptive baseline (ADC counts)."""
        nid = node_id or self.default_node
        return self._sdo.read_u16(nid, OD.COLLISION_MEAN, 0)

    def get_sigma(self, node_id: Optional[int] = None) -> int:
        """Poll the AUTO-mode robust noise scale (ADC counts)."""
        nid = node_id or self.default_node
        return self._sdo.read_u16(nid, OD.COLLISION_SIGMA, 0)


# ============================================================================
# State / system queries
# ============================================================================

class State:
    """System state queries."""

    def __init__(self, sdo: SdoClient, listener: TpdoListener):
        self._sdo = sdo
        self._listener = listener

    def get_uptime(self, node_id: int) -> int:
        """Read uptime in seconds from a slave."""
        return self._sdo.read_u32(node_id, OD.UPTIME_SECONDS, 0)

    def get_free_heap(self, node_id: int) -> int:
        return self._sdo.read_u32(node_id, OD.FREE_HEAP_BYTES, 0)

    def get_cpu_temperature(self, node_id: int) -> float:
        """Read CPU temperature in °C (reported as centidegrees on the wire)."""
        raw = self._sdo.read(node_id, OD.CPU_TEMPERATURE, 0, 2)
        return struct.unpack("<h", raw)[0] / 100.0

    def get_can_errors(self, node_id: int) -> int:
        """Read the slave's cumulative CAN error counter."""
        return self._sdo.read_u32(node_id, OD.CAN_ERROR_COUNTER, 0)

    def get_enabled_modules(self, node_id: int) -> int:
        """Read the enabled-modules bitmask (see firmware ENABLED_MODULES bits)."""
        return self._sdo.read_u32(node_id, OD.ENABLED_MODULES, 0)

    def reboot(self, node_id: int):
        """Send reboot command to a slave."""
        self._sdo.write_u8(node_id, OD.REBOOT_COMMAND, 0, 1)

    def set_node_id(self, node_id: int, new_node_id: int):
        """Reassign a slave's CAN node id (1..127).

        The slave persists the new id to NVS and performs a CANopen
        communication reset. The SDO response is sent on the OLD id first; after
        the reset the node reappears at `new_node_id`.
        """
        if not 1 <= new_node_id <= 127:
            raise ValueError("new_node_id must be in 1..127")
        self._sdo.write_u8(node_id, OD.COMMANDED_NODE_ID, 0, new_node_id)

    def scan_nodes(self, timeout: float = 3.0) -> list[int]:
        """Scan for online nodes by listening for heartbeats and TPDOs."""
        return self._listener.scan(timeout)

    def send_nmt(self, node_id: int, command: int):
        """Send raw NMT command. node_id=0 broadcasts."""
        msg = can.Message(
            arbitration_id=0x000,
            data=bytes([command, node_id]),
            is_extended_id=False,
        )
        self._sdo.bus.send(msg)

    def start_node(self, node_id: int = 0):
        self.send_nmt(node_id, 0x01)

    def stop_node(self, node_id: int = 0):
        self.send_nmt(node_id, 0x02)

    def reset_node(self, node_id: int = 0):
        self.send_nmt(node_id, 0x81)

    def send_sync(self):
        """Send one CANopen SYNC (COB-ID 0x080, no data).

        Bench tool for the strobed sweep: strobing illumination nodes flash and
        latching motor nodes report their position on every SYNC. In a real
        sweep the ESP32 master sends SYNC so the camera trigger is aligned.
        """
        msg = can.Message(arbitration_id=SYNC_COB_ID, data=b"", is_extended_id=False)
        self._sdo.bus.send(msg)


# ============================================================================
# Main client — the public API
# ============================================================================

class UC2Client:
    """
    Main entry point for controlling UC2 hardware over CANopen.

    Similar to UC2-REST's UC2Client but communicates over CAN instead of serial.

    Args:
        interface: CAN transport — "socketcan" for a Linux SocketCAN device
            (e.g. an MCP2515 SPI HAT that enumerates as `can0`), or "waveshare"
            for a USB-CAN-A serial adapter. If None (default), picks "waveshare"
            when `port` is given, otherwise "socketcan".
        channel: SocketCAN interface name (default "can0"). Used when
            interface == "socketcan".
        port: serial port of the Waveshare USB-CAN-A adapter (auto-detected if
            None). Used when interface == "waveshare".
        bitrate: CAN bus bitrate. For SocketCAN this is configured at the OS
            level (`ip link set <channel> up type can bitrate <N>`) and ignored
            here; for the Waveshare adapter it is applied. Must match firmware.
        serial_baudrate: Waveshare adapter serial speed (default 2 Mbit/s)
        sdo_timeout: SDO transfer timeout in seconds

    Example:
        # MCP2515 HAT (after `sudo ip link set can0 up type can bitrate 500000`)
        uc2 = UC2Client()                       # interface="socketcan", channel="can0"
        # or a Waveshare USB-CAN-A adapter:
        # uc2 = UC2Client(port="/dev/ttyUSB0")
        uc2.motor.move(axis=0, position=1000, speed=20000, node_id=10)
        uc2.motor.wait_for_idle(axis=0, node_id=10)
        pos = uc2.motor.get_position(axis=0, node_id=10)
        print(f"Motor at {pos} steps")
        uc2.laser.set_value(channel=0, pwm=512, node_id=20)
        uc2.led.fill(r=255, g=0, b=0, node_id=20)
        uc2.close()
    """

    def __init__(
        self,
        port: Optional[str] = None,
        bitrate: int = 500_000,
        serial_baudrate: int = 2_000_000,
        sdo_timeout: float = 2.0,
        sdo_settle_s: float = 0.010,
        interface: Optional[str] = None,
        channel: Optional[str] = None,
        log_level: int = logging.WARNING,
        debug: bool = False
    ):
        # Configure the package-wide logger so all sub-modules (sdo, waveshare_bus)
        # honour the same level. The caller controls verbosity; we never install
        # a handler — that is the application's responsibility.
        logging.getLogger("uc2canopen").setLevel(log_level)

        # Pick a transport. Default to SocketCAN (e.g. an MCP2515 HAT) unless a
        # Waveshare serial port was given.
        if interface is None:
            interface = "waveshare" if port is not None else "socketcan"

        # Open the CAN bus
        if interface == "socketcan":
            # MCP2515 HAT etc. The bitrate is configured at the OS level with
            # `ip link set <channel> up type can bitrate <N>`, not here.
            self._bus = can.interface.Bus(
                interface="socketcan",
                channel=channel or "can0",
            )
        elif interface == "waveshare":
            # Auto-detect the serial port if not specified.
            if port is None:
                port = find_waveshare_port()
                if port is None:
                    raise RuntimeError(
                        "No Waveshare USB-CAN-A adapter found. "
                        "Pass port= explicitly or check USB connection."
                    )
            self._bus = WaveshareBus(
                channel=port,
                bitrate=bitrate,
                serial_baudrate=serial_baudrate,
            )
        else:
            raise ValueError(
                f"Unknown interface {interface!r}; use 'socketcan' or 'waveshare'."
            )

        # Create transport layers (both implement can.Listener; the shared
        # Notifier dispatches every received frame to BOTH so SDO and PDO
        # paths do not race for `bus.recv()`).
        self._sdo = SdoClient(self._bus, timeout_s=sdo_timeout,
                              settle_s=sdo_settle_s)
        self._listener = TpdoListener(self._bus)
        self._notifier = can.Notifier(self._bus, [self._sdo, self._listener])

        # Create high-level device modules
        self.motor = Motor(self._sdo, self._listener)
        self.laser = Laser(self._sdo)
        self.led = Led(self._sdo)
        self.galvo = Galvo(self._sdo)
        self.pid = Pid(self._sdo)
        self.encoder = Encoder(self._sdo)
        self.digital = DigitalIO(self._sdo)
        self.analog = Analog(self._sdo)
        self.collision = Collision(self._sdo)
        self.state = State(self._sdo, self._listener)

        # Expose the raw bus for advanced use
        self.bus = self._bus

    # --- strobed sweep bench helpers ---------------------------------------
    def send_sync(self):
        """Send one CANopen SYNC (see State.send_sync)."""
        self.state.send_sync()

    def on_sync_position(self, callback):
        """Register callback(node_id, position_steps, sync_count) for TPDO3 latch frames."""
        self._listener.on_sync_position(callback)

    def close(self):
        """Shut down the notifier and close the CAN bus."""
        try:
            self._notifier.stop(timeout=2.0)
        except Exception:
            pass
        self._bus.shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def __repr__(self):
        chan = getattr(self._bus, "channel", None) or getattr(self._bus, "channel_info", "?")
        return f"UC2Client(channel={chan!r}, bus={type(self._bus).__name__})"
