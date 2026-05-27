#!/usr/bin/env python3
"""
UC2 CANopen motor control example.

Demonstrates the high-level Python API for controlling UC2 motors,
lasers, and LEDs via a Waveshare USB-CAN-A adapter.

This replaces the old UC2-REST serial JSON interface with direct
CANopen SDO communication — same API shape, different transport.

Usage:
    # Install the package first:
    uv pip install -e .
    # Auto-detect adapter:
    python src/motor_demo.py
    # Specify port + nodes:
    python /Users/bene/Dropbox/Dokumente/Promotion/PROJECTS/UC2-REST-CANOPEN/src/motor_demo.py --port /dev/cu.usbserial-10 --motor-node 11 
"""

from __future__ import annotations

import argparse
import sys
import time
import traceback

from uc2canopen import NODE, SdoError, UC2Client


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="UC2 CANopen motor/laser/LED demo")
    p.add_argument(
        "--port", "-p", default=None,
        help="Waveshare USB-CAN-A serial port. Default: auto-detect "
             "(scans /dev/ttyUSB*, /dev/ttyACM*, /dev/cu.usbserial-*, /dev/cu.wchusbserial*).",
    )
    p.add_argument("--bitrate", "-b", type=int, default=500_000,
                   help="CAN bus bitrate (default 500000 — must match firmware)")
    p.add_argument("--motor-node", type=int, default=NODE.MOT_X,
                   help=f"Motor CAN node ID (default {NODE.MOT_X} = NODE.MOT_X)")
    p.add_argument("--laser-node", type=int, default=NODE.LED_0,
                   help=f"Laser/LED CAN node ID (default {NODE.LED_0} = combined illum. board)")
    p.add_argument("--axis", type=int, default=0,
                   help="Motor axis index on the slave (default 0)")
    p.add_argument("--steps", type=int, default=3000,
                   help="Relative move distance in steps (default 3000)")
    p.add_argument("--speed", type=int, default=20000,
                   help="Move speed in steps/s (default 20000)")
    p.add_argument("--sdo-settle-ms", type=float, default=10.0,
                   help="Inter-SDO settle time (ms). Increase if you see SDO timeouts.")
    p.add_argument("--debug", action="store_true",
                   help="Verbose: print every raw serial byte and CAN frame")
    p.add_argument("--skip-motor", action="store_true", help="Skip motor moves")
    p.add_argument("--skip-laser", action="store_true", help="Skip laser test")
    p.add_argument("--skip-led", action="store_true", help="Skip LED test")
    return p.parse_args()


def _step(label: str, fn, *args, **kwargs) -> bool:
    """Run one demo step, catching SdoError and other failures so later steps still run."""
    print(f"\n--- {label} ---")
    try:
        fn(*args, **kwargs)
        return True
    except SdoError as e:
        print(f"  ✗ SDO failure: {e}", file=sys.stderr)
    except Exception as e:
        print(f"  ✗ {type(e).__name__}: {e}", file=sys.stderr)
        if "--debug" in sys.argv:
            traceback.print_exc()
    return False


def _run_motor(uc2: UC2Client, args: argparse.Namespace) -> None:
    print(f"Moving motor on node {args.motor_node} axis {args.axis}: "
          f"{args.steps:+d} steps at {args.speed} steps/s")
    uc2.motor.move(axis=args.axis, position=args.steps, speed=args.speed,
                   node_id=args.motor_node)
    ok = uc2.motor.wait_for_idle(axis=args.axis, node_id=args.motor_node, timeout=10.0)
    pos = uc2.motor.get_position(axis=args.axis, node_id=args.motor_node)
    print(f"  {'✓ Done' if ok else '⚠ Timeout'} — position: {pos} steps")

    print(f"Moving back: {-args.steps:+d} steps")
    uc2.motor.move(axis=args.axis, position=-args.steps, speed=args.speed,
                   node_id=args.motor_node)
    ok = uc2.motor.wait_for_idle(axis=args.axis, node_id=args.motor_node, timeout=10.0)
    pos = uc2.motor.get_position(axis=args.axis, node_id=args.motor_node)
    print(f"  {'✓ Done' if ok else '⚠ Timeout'} — position: {pos} steps")


def _run_laser(uc2: UC2Client, args: argparse.Namespace) -> None:
    print(f"Laser channel 0 on node {args.laser_node}: PWM 512 for 2s")
    uc2.laser.set_value(channel=0, pwm=512, node_id=args.laser_node)
    time.sleep(2.0)
    uc2.laser.off(channel=0, node_id=args.laser_node)
    print("  ✓ Laser off")


def _run_led(uc2: UC2Client, args: argparse.Namespace) -> None:
    print(f"LED fill red on node {args.laser_node} for 1s")
    uc2.led.fill(r=255, g=0, b=0, node_id=args.laser_node)
    time.sleep(1.0)
    uc2.led.off(node_id=args.laser_node)
    print("  ✓ LED off")


def _run_status(uc2: UC2Client, node_id: int) -> None:
    uptime = uc2.state.get_uptime(node_id)
    heap = uc2.state.get_free_heap(node_id)
    print(f"  Node {node_id}: uptime={uptime}s, free_heap={heap} bytes")


def main() -> int:
    args = _parse_args()

    try:
        uc2 = UC2Client(
            port=args.port,
            bitrate=args.bitrate,
            sdo_settle_s=args.sdo_settle_ms / 1000.0,
            debug=args.debug,
        )
    except (RuntimeError, OSError) as e:
        print(f"Failed to open CAN bus: {e}", file=sys.stderr)
        return 1
    print(f"Connected: {uc2}")

    try:
        _step("Scanning for nodes",
              lambda: print(f"  Found: {uc2.state.scan_nodes(timeout=2.0)}"))

        if not args.skip_motor:
            _step(f"Motor test on node {args.motor_node}",
                  _run_motor, uc2, args)

        if not args.skip_laser:
            _step(f"Laser test on node {args.laser_node}",
                  _run_laser, uc2, args)

        if not args.skip_led:
            _step(f"LED test on node {args.laser_node}",
                  _run_led, uc2, args)

        _step(f"Status of node {args.motor_node}",
              _run_status, uc2, args.motor_node)
    finally:
        uc2.close()
        print("\nDone.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
