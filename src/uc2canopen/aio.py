"""
Asyncio facade for uc2canopen.

Wraps the blocking `UC2Client` so it can be driven from asyncio applications
(e.g. the newswitch backend) without blocking the event loop. Every blocking
call is executed via `asyncio.to_thread`; asynchronous hardware events
(TPDO frames, heartbeats) arriving on the python-can notifier thread are
bridged onto the event loop and exposed as a typed async event stream.

This module is purely additive: the synchronous `UC2Client` API is unchanged
and remains the interface consumed by ImSwitch.

Usage:
    from uc2canopen.aio import AsyncUC2Client

    uc2 = await AsyncUC2Client.create(channel="can0")
    await uc2.move(axis=0, position=1000, speed=20000, node_id=11)
    await uc2.wait_for_idle(axis=0, node_id=11)

    async for event in uc2.events():
        print(event)

    await uc2.aclose()
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass
from typing import Any, AsyncIterator, Callable, Optional, Union

from .client import MotorState, NodeInfo, UC2Client

_log = logging.getLogger(__name__)

# Default per-subscriber queue size. When a slow consumer falls behind, the
# oldest events are dropped (position streams are idempotent snapshots).
_DEFAULT_QUEUE_SIZE = 1024


# ---------------------------------------------------------------------------
# Typed events
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class MotorUpdateEvent:
    """Periodic motor snapshot from a TPDO1 frame."""

    node_id: int
    axis: int
    position: int
    running: bool
    homing_status: int


@dataclass(frozen=True)
class MotorDoneEvent:
    """A motor transitioned from running to stopped."""

    node_id: int
    axis: int
    position: int


@dataclass(frozen=True)
class HomingChangedEvent:
    """The homing phase of an axis changed (0=idle, 1=homing, 2=done, 3=timeout)."""

    node_id: int
    axis: int
    status: int


@dataclass(frozen=True)
class HeartbeatEvent:
    """A CANopen heartbeat was received from a node."""

    node_id: int
    nmt_state: int


CanEvent = Union[MotorUpdateEvent, MotorDoneEvent, HomingChangedEvent, HeartbeatEvent]


# ---------------------------------------------------------------------------
# Async client
# ---------------------------------------------------------------------------

class AsyncUC2Client:
    """Asyncio wrapper around the blocking `UC2Client`.

    Construct with `await AsyncUC2Client.create(...)` (same kwargs as
    `UC2Client`). The underlying sync client is available as `.sync` for
    anything not wrapped explicitly; `await client.call(fn, ...)` runs any
    blocking callable off-loop.
    """

    def __init__(self, client: UC2Client, loop: Optional[asyncio.AbstractEventLoop] = None):
        self._client = client
        self._loop = loop or asyncio.get_running_loop()
        self._queues: list[asyncio.Queue[CanEvent]] = []
        self._motion_waiters: dict[tuple[int, int], list[asyncio.Future[int]]] = {}
        self._closed = False

        listener = client._listener
        listener.on_motor_update(self._on_motor_update)
        listener.on_motor_done(self._on_motor_done)
        listener.on_homing_changed(self._on_homing_changed)
        listener.on_heartbeat(self._on_heartbeat)

    @classmethod
    async def create(cls, **kwargs: Any) -> "AsyncUC2Client":
        """Open the CAN bus off-loop and return a ready async client.

        Accepts the same keyword arguments as `UC2Client` (port, bitrate,
        interface, channel, sdo_timeout, ...).
        """
        loop = asyncio.get_running_loop()
        client = await asyncio.to_thread(UC2Client, **kwargs)
        return cls(client, loop=loop)

    # -- plumbing -----------------------------------------------------------

    @property
    def sync(self) -> UC2Client:
        """The wrapped synchronous client (full uc2canopen API surface)."""
        return self._client

    async def call(self, fn: Callable[..., Any], /, *args: Any, **kwargs: Any) -> Any:
        """Run any blocking callable (e.g. `client.sync.motor.get_position`) off-loop."""
        return await asyncio.to_thread(fn, *args, **kwargs)

    async def aclose(self) -> None:
        """Shut down the notifier and close the CAN bus."""
        self._closed = True
        await asyncio.to_thread(self._client.close)

    async def __aenter__(self) -> "AsyncUC2Client":
        return self

    async def __aexit__(self, *args: Any) -> None:
        await self.aclose()

    # -- event bridge (notifier thread -> event loop) ------------------------

    def _publish(self, event: CanEvent) -> None:
        """Thread-safe: schedule fan-out of an event onto the loop."""
        if self._closed:
            return
        try:
            self._loop.call_soon_threadsafe(self._publish_on_loop, event)
        except RuntimeError:
            # Event loop already closed during shutdown.
            pass

    def _publish_on_loop(self, event: CanEvent) -> None:
        for queue in list(self._queues):
            if queue.full():
                try:
                    queue.get_nowait()  # drop oldest
                except asyncio.QueueEmpty:
                    pass
            queue.put_nowait(event)
        if isinstance(event, MotorDoneEvent):
            waiters = self._motion_waiters.pop((event.node_id, event.axis), [])
            for fut in waiters:
                if not fut.done():
                    fut.set_result(event.position)

    def _on_motor_update(self, node_id: int, axis: int, state: MotorState) -> None:
        self._publish(MotorUpdateEvent(
            node_id=node_id, axis=axis, position=state.position,
            running=state.running, homing_status=state.homing_status,
        ))

    def _on_motor_done(self, node_id: int, axis: int, position: int) -> None:
        self._publish(MotorDoneEvent(node_id=node_id, axis=axis, position=position))

    def _on_homing_changed(self, node_id: int, axis: int, status: int) -> None:
        self._publish(HomingChangedEvent(node_id=node_id, axis=axis, status=status))

    def _on_heartbeat(self, node_id: int, info: NodeInfo) -> None:
        self._publish(HeartbeatEvent(node_id=node_id, nmt_state=info.nmt_state))

    async def events(self, queue_size: int = _DEFAULT_QUEUE_SIZE) -> AsyncIterator[CanEvent]:
        """Yield typed hardware events as they arrive.

        Multiple consumers may iterate concurrently; each gets its own queue.
        The oldest events are dropped if a consumer falls behind.
        """
        queue: asyncio.Queue[CanEvent] = asyncio.Queue(maxsize=queue_size)
        self._queues.append(queue)
        try:
            while True:
                yield await queue.get()
        finally:
            self._queues.remove(queue)

    # -- motor ---------------------------------------------------------------

    async def move(self, axis: int = 0, position: int = 0, speed: int = 20000,
                   acceleration: int = 0, is_absolute: bool = False,
                   is_forever: bool = False, node_id: Optional[int] = None) -> None:
        """Start a motor move (see `Motor.move`). Returns once the command is issued."""
        await asyncio.to_thread(
            self._client.motor.move, axis=axis, position=position, speed=speed,
            acceleration=acceleration, is_absolute=is_absolute,
            is_forever=is_forever, node_id=node_id,
        )

    async def move_and_wait(self, axis: int = 0, position: int = 0, speed: int = 20000,
                            acceleration: int = 0, is_absolute: bool = False,
                            node_id: Optional[int] = None,
                            timeout: float = 60.0) -> int:
        """Start a move and wait (event-driven) until the motor stops.

        Returns the final position. Falls back to polled `wait_for_idle` if no
        TPDO done-event arrives within `timeout`. On cancellation the motor is
        stopped before the CancelledError propagates.
        """
        nid = node_id if node_id is not None else self._client.motor.default_node
        fut: asyncio.Future[int] = self._loop.create_future()
        self._motion_waiters.setdefault((nid, axis), []).append(fut)
        try:
            await self.move(axis=axis, position=position, speed=speed,
                            acceleration=acceleration, is_absolute=is_absolute,
                            node_id=nid)
            try:
                return await asyncio.wait_for(fut, timeout=timeout)
            except asyncio.TimeoutError:
                _log.warning("No TPDO done-event within %.1fs; falling back to polling", timeout)
                await asyncio.to_thread(self._client.motor.wait_for_idle, axis, nid, 5.0)
                return await self.get_position(axis=axis, node_id=nid)
        except asyncio.CancelledError:
            await asyncio.shield(self.stop(axis=axis, node_id=nid))
            raise
        finally:
            waiters = self._motion_waiters.get((nid, axis))
            if waiters and fut in waiters:
                waiters.remove(fut)

    async def stop(self, axis: int = 0, node_id: Optional[int] = None) -> None:
        """Stop a motor axis."""
        await asyncio.to_thread(self._client.motor.stop, axis, node_id)

    async def get_position(self, axis: int = 0, node_id: Optional[int] = None) -> int:
        """Read a motor position (TPDO cache preferred, SDO fallback)."""
        return await asyncio.to_thread(self._client.motor.get_position, axis, node_id)

    async def is_running(self, axis: int = 0, node_id: Optional[int] = None) -> bool:
        """Check whether a motor axis is currently running."""
        return await asyncio.to_thread(self._client.motor.is_running, axis, node_id)

    async def home(self, axis: int = 0, speed: int = 15000, direction: int = -1,
                   timeout_ms: int = 20000, node_id: Optional[int] = None) -> None:
        """Start homing for an axis (see `Motor.home`)."""
        await asyncio.to_thread(self._client.motor.home, axis, speed, direction,
                                timeout_ms, node_id)

    async def home_and_wait(self, axis: int = 0, speed: int = 15000, direction: int = -1,
                            timeout_ms: int = 20000, node_id: Optional[int] = None,
                            timeout: float = 30.0) -> bool:
        """Home an axis and wait for completion. Returns True if homed."""
        await self.home(axis=axis, speed=speed, direction=direction,
                        timeout_ms=timeout_ms, node_id=node_id)
        return await asyncio.to_thread(self._client.motor.wait_for_homed, axis,
                                       node_id, timeout)

    # -- laser / led ----------------------------------------------------------

    async def laser_set(self, channel: int = 0, pwm: int = 0,
                        node_id: Optional[int] = None) -> None:
        """Set a laser channel PWM value."""
        await asyncio.to_thread(self._client.laser.set_value, channel, pwm, node_id)

    async def laser_get(self, channel: int = 0, node_id: Optional[int] = None) -> int:
        """Read back a laser channel PWM value."""
        return await asyncio.to_thread(self._client.laser.get_value, channel, node_id)

    async def led_fill(self, r: int = 0, g: int = 0, b: int = 0,
                       node_id: Optional[int] = None) -> None:
        """Fill the LED array with a uniform colour."""
        await asyncio.to_thread(self._client.led.fill, r, g, b, node_id)

    async def led_off(self, node_id: Optional[int] = None) -> None:
        """Turn the LED array off."""
        await asyncio.to_thread(self._client.led.off, node_id)

    # -- galvo ----------------------------------------------------------------

    async def galvo_goto(self, x: int, y: int, node_id: Optional[int] = None) -> None:
        """Move the galvo to an absolute XY position (DAC counts)."""
        await asyncio.to_thread(self._client.galvo.set_position, x, y, node_id)

    async def galvo_raster_scan(self, node_id: Optional[int] = None, **kwargs: Any) -> None:
        """Configure and start a raster scan (see `Galvo.raster_scan` for kwargs)."""
        await asyncio.to_thread(
            lambda: self._client.galvo.raster_scan(node_id=node_id, **kwargs)
        )

    async def galvo_stop(self, node_id: Optional[int] = None) -> None:
        """Stop any active galvo scan / motion."""
        await asyncio.to_thread(self._client.galvo.stop, node_id)

    async def galvo_status(self, node_id: Optional[int] = None) -> dict:
        """Read and decode the galvo status word."""
        return await asyncio.to_thread(self._client.galvo.get_status, node_id)

    # -- system ---------------------------------------------------------------

    async def scan_nodes(self, timeout: float = 3.0) -> list[int]:
        """Passively scan for online nodes (heartbeats/TPDOs) for `timeout` seconds."""
        return await asyncio.to_thread(self._client.state.scan_nodes, timeout)

    async def reboot(self, node_id: int) -> None:
        """Reboot a slave node."""
        await asyncio.to_thread(self._client.state.reboot, node_id)
