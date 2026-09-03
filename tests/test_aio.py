"""Tests for uc2canopen.aio — asyncio facade + TpdoListener event bridging.

Hardware-free: synthetic `can.Message` frames are pushed straight into a
`TpdoListener`, and a stub client stands in for the SDO side, so we can
assert the full event pipeline (TPDO/heartbeat -> typed async events) and
`move_and_wait`'s done-edge / cancellation semantics.

Run:
    pytest tests/test_aio.py -v
    # or, without pytest installed:
    python tests/test_aio.py
"""

from __future__ import annotations

import asyncio
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import can  # noqa: E402

from uc2canopen.aio import (  # noqa: E402
    AsyncUC2Client,
    HeartbeatEvent,
    HomingChangedEvent,
    MotorDoneEvent,
    MotorUpdateEvent,
)
from uc2canopen.client import TpdoListener  # noqa: E402


def tpdo1_frame(node_id: int, position: int, running: bool, homing: int = 0) -> can.Message:
    """Build a synthetic TPDO1 frame (i32 position + u8 status + u8 homing)."""
    data = struct.pack("<i", position) + bytes([1 if running else 0, homing])
    return can.Message(arbitration_id=0x180 + node_id, data=data, is_extended_id=False)


def heartbeat_frame(node_id: int, nmt_state: int = 0x05) -> can.Message:
    """Build a synthetic heartbeat frame."""
    return can.Message(arbitration_id=0x700 + node_id, data=bytes([nmt_state]), is_extended_id=False)


class StubMotor:
    """Records move/stop calls; no I/O."""

    default_node = 11

    def __init__(self):
        self.moves = []
        self.stops = []

    def move(self, **kwargs):
        self.moves.append(kwargs)

    def stop(self, axis=0, node_id=None):
        self.stops.append((axis, node_id))

    def wait_for_idle(self, axis=0, node_id=None, timeout=5.0):
        return True

    def get_position(self, axis=0, node_id=None):
        return 0


class StubClient:
    """Stands in for UC2Client: a real TpdoListener + recording motor."""

    def __init__(self):
        self._listener = TpdoListener(bus=None)
        self.motor = StubMotor()

    def close(self):
        pass


def test_events_bridge_tpdo_and_heartbeat():
    """TPDO1/heartbeat frames come out of events() as typed async events."""

    async def scenario():
        stub = StubClient()
        client = AsyncUC2Client(stub)  # type: ignore[arg-type]

        received = []

        async def collect():
            # frame1 -> update; frame2 -> update + homing-change + done-edge;
            # heartbeat -> 1. Five events total.
            async for event in client.events():
                received.append(event)
                if len(received) >= 5:
                    return

        collector = asyncio.create_task(collect())
        await asyncio.sleep(0)  # let the collector subscribe

        listener = stub._listener
        listener.on_message_received(tpdo1_frame(11, 100, running=True))
        listener.on_message_received(tpdo1_frame(11, 500, running=False, homing=2))
        listener.on_message_received(heartbeat_frame(12))
        await asyncio.wait_for(collector, timeout=2.0)
        return received

    received = asyncio.run(scenario())

    updates = [e for e in received if isinstance(e, MotorUpdateEvent)]
    assert len(updates) == 2 and updates[0].position == 100 and updates[0].running
    dones = [e for e in received if isinstance(e, MotorDoneEvent)]
    assert len(dones) == 1 and dones[0].position == 500 and dones[0].node_id == 11
    homings = [e for e in received if isinstance(e, HomingChangedEvent)]
    assert len(homings) == 1 and homings[0].status == 2
    beats = [e for e in received if isinstance(e, HeartbeatEvent)]
    assert len(beats) == 1 and beats[0].node_id == 12


def test_move_and_wait_resolves_on_done_edge():
    """move_and_wait returns the final position when the running->stopped edge arrives."""

    async def scenario():
        stub = StubClient()
        client = AsyncUC2Client(stub)  # type: ignore[arg-type]

        async def inject_done():
            await asyncio.sleep(0.05)
            stub._listener.on_message_received(tpdo1_frame(11, 0, running=True))
            stub._listener.on_message_received(tpdo1_frame(11, 1234, running=False))

        injector = asyncio.create_task(inject_done())
        final = await client.move_and_wait(axis=0, position=1234, node_id=11, timeout=2.0)
        await injector
        return final, stub.motor.moves

    final, moves = asyncio.run(scenario())
    assert final == 1234
    assert len(moves) == 1 and moves[0]["position"] == 1234


def test_move_and_wait_stops_motor_on_cancellation():
    """Cancelling a pending move sends a stop to the hardware."""

    async def scenario():
        stub = StubClient()
        client = AsyncUC2Client(stub)  # type: ignore[arg-type]

        task = asyncio.create_task(
            client.move_and_wait(axis=0, position=99, node_id=11, timeout=10.0)
        )
        await asyncio.sleep(0.05)
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        return stub.motor.stops

    stops = asyncio.run(scenario())
    assert stops == [(0, 11)]


def test_callbacks_fire_outside_listener_lock():
    """A callback may safely read back cached state (regression: lock deadlock)."""
    listener = TpdoListener(bus=None)
    seen = []

    def callback(node_id, axis, state):
        # This get_motor() call would deadlock if callbacks fired under the lock.
        seen.append(listener.get_motor(node_id, axis))

    listener.on_motor_update(callback)
    listener.on_message_received(tpdo1_frame(11, 42, running=False))
    assert len(seen) == 1 and seen[0].position == 42


def _main() -> int:
    """Self-runner (no pytest needed)."""
    failures = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except Exception as exc:  # noqa: BLE001
                failures += 1
                print(f"FAIL {name}: {exc!r}")
    print(f"{'OK' if not failures else 'FAILED'}")
    return failures


if __name__ == "__main__":
    raise SystemExit(_main())
