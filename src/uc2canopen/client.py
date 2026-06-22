"""
UC2 CANopen Client — high-level Python interface for openUC2 hardware.

Mirrors the UC2-REST API shape (uc2.motor, uc2.laser, uc2.led, uc2.state)
but communicates over CAN bus via SDO instead of JSON-over-serial.

Usage:
    from uc2canopen import UC2Client
    uc2 = UC2Client(port="/dev/ttyUSB0")
    uc2.motor.move(axis=1, position=1000, speed=20000)
    uc2.motor.wait_for_idle(axis=1)
    print(uc2.motor.get_position(axis=1))
    uc2.laser.set_value(channel=0, pwm=512)
    uc2.led.fill(r=255, g=0, b=0)
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
HB_BASE = 0x700


# ============================================================================
# TPDO listener — background thread that caches slave state
# ============================================================================

@dataclass
class MotorState:
    position: int = 0
    running: bool = False
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
        self._callbacks: list[Callable] = []

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

    def on_message_received(self, msg: can.Message) -> None:
        cob = msg.arbitration_id
        fc = cob & 0x780
        node_id = cob & 0x07F

        if fc == TPDO1_BASE and len(msg.data) >= 5:
                # TPDO1: i32 position (4 bytes) + u8 status (1 byte)
            pos = struct.unpack_from("<i", msg.data, 0)[0]
            status = msg.data[4]
            running = bool(status & 0x01)

            with self._lock:
                key = (node_id, 0)  # sub 0 for now; multi-axis slaves use sub mapping
                prev = self.motors.get(key)
                was_running = prev.running if prev and prev.seen else False

                self.motors[key] = MotorState(
                    position=pos,
                    running=running,
                    last_update=time.time(),
                    seen=True,
                )

                if was_running and not running:
                    for cb in self._callbacks:
                        try:
                            cb(node_id, 0, pos)
                        except Exception:
                            pass

        elif fc == HB_BASE and len(msg.data) >= 1:
            with self._lock:
                self.nodes[node_id] = NodeInfo(
                    node_id=node_id,
                    nmt_state=msg.data[0],
                    last_heartbeat=time.time(),
                    seen=True,
                )

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

    def reboot(self, node_id: int):
        """Send reboot command to a slave."""
        self._sdo.write_u8(node_id, OD.REBOOT_COMMAND, 0, 1)

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
        self.state = State(self._sdo, self._listener)

        # Expose the raw bus for advanced use
        self.bus = self._bus

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
