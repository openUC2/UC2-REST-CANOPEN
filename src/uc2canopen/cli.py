"""
uc2can CLI — command-line tool for UC2 CANopen control.

Usage:
    uc2can scan                          # find nodes on the bus
    uc2can move --node 10 --pos 1000     # move motor
    uc2can laser --node 20 --ch 0 --pwm 512
    uc2can led --node 20 --r 255 --g 0 --b 0
    uc2can status --node 10              # read uptime, position
    uc2can sniff                         # dump raw CAN frames
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

import can

from .client import UC2Client
from .od import NODE
from .waveshare_bus import WaveshareBus, find_waveshare_port


def cmd_scan(args):
    with UC2Client(port=args.port, bitrate=args.bitrate) as uc2:
        print(f"Scanning for nodes ({args.timeout}s)...")
        nodes = uc2.state.scan_nodes(timeout=args.timeout)
        if nodes:
            print(f"Found {len(nodes)} node(s): {nodes}")
        else:
            print("No nodes found. Check CAN wiring and bitrate.")


def cmd_move(args):
    with UC2Client(port=args.port, bitrate=args.bitrate) as uc2:
        print(f"Moving motor: node={args.node} axis={args.axis} "
              f"pos={args.pos} speed={args.speed} abs={args.abs_}")
        uc2.motor.move(
            axis=args.axis,
            position=args.pos,
            speed=args.speed,
            is_absolute=args.abs_,
            node_id=args.node,
        )
        if args.wait:
            print("Waiting for motor to finish...")
            ok = uc2.motor.wait_for_idle(args.axis, args.node, timeout=args.timeout)
            pos = uc2.motor.get_position(args.axis, args.node)
            print(f"{'Done' if ok else 'Timeout'}. Position: {pos}")
        else:
            print("Command sent. Use --wait to block until done.")


def cmd_stop(args):
    with UC2Client(port=args.port, bitrate=args.bitrate) as uc2:
        uc2.motor.stop(axis=args.axis, node_id=args.node)
        print(f"Stop sent to node {args.node} axis {args.axis}")


def cmd_home(args):
    with UC2Client(port=args.port, bitrate=args.bitrate) as uc2:
        print(f"Homing: node={args.node} axis={args.axis}")
        uc2.motor.home(
            axis=args.axis,
            speed=args.speed,
            direction=args.direction,
            node_id=args.node,
        )
        if args.wait:
            ok = uc2.motor.wait_for_idle(args.axis, args.node, timeout=args.timeout)
            print(f"{'Homed' if ok else 'Timeout'}")


def cmd_laser(args):
    with UC2Client(port=args.port, bitrate=args.bitrate) as uc2:
        uc2.laser.set_value(channel=args.ch, pwm=args.pwm, node_id=args.node)
        print(f"Laser ch={args.ch} pwm={args.pwm} on node {args.node}")


def cmd_led(args):
    with UC2Client(port=args.port, bitrate=args.bitrate) as uc2:
        if args.off:
            uc2.led.off(node_id=args.node)
            print(f"LED off on node {args.node}")
        else:
            uc2.led.fill(r=args.r, g=args.g, b=args.b, node_id=args.node)
            print(f"LED fill ({args.r},{args.g},{args.b}) on node {args.node}")


def cmd_status(args):
    with UC2Client(port=args.port, bitrate=args.bitrate) as uc2:
        try:
            uptime = uc2.state.get_uptime(args.node)
            heap = uc2.state.get_free_heap(args.node)
            pos = uc2.motor.get_position(axis=0, node_id=args.node)
            running = uc2.motor.is_running(axis=0, node_id=args.node)
            print(f"Node {args.node}:")
            print(f"  Uptime:    {uptime}s")
            print(f"  Free heap: {heap} bytes")
            print(f"  Motor pos: {pos} steps")
            print(f"  Running:   {running}")
        except Exception as e:
            print(f"Error reading node {args.node}: {e}")


def cmd_sniff(args):
    port = args.port or find_waveshare_port()
    if not port:
        print("No Waveshare adapter found.", file=sys.stderr)
        sys.exit(1)

    bus = WaveshareBus(channel=port, bitrate=args.bitrate)
    print(f"Sniffing CAN bus on {port} @ {args.bitrate // 1000}k. Ctrl+C to stop.\n")

    FC_NAMES = {
        0x000: "NMT", 0x080: "SYNC", 0x180: "TPDO1", 0x200: "RPDO1",
        0x280: "TPDO2", 0x300: "RPDO2", 0x380: "TPDO3", 0x400: "RPDO3",
        0x580: "SDO↑", 0x600: "SDO↓", 0x700: "HB",
    }

    try:
        while True:
            msg = bus.recv(timeout=1.0)
            if msg is None:
                continue
            fc = msg.arbitration_id & 0x780
            nid = msg.arbitration_id & 0x07F
            name = FC_NAMES.get(fc, f"0x{fc:03X}")
            data_hex = " ".join(f"{b:02X}" for b in msg.data)
            print(f"[{msg.timestamp:.3f}] 0x{msg.arbitration_id:03X} "
                  f"{name:6s} n={nid:3d}  [{msg.dlc}] {data_hex}")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        bus.shutdown()


def cmd_reboot(args):
    with UC2Client(port=args.port, bitrate=args.bitrate) as uc2:
        uc2.state.reboot(args.node)
        print(f"Reboot command sent to node {args.node}")


def main():
    p = argparse.ArgumentParser(
        prog="uc2can",
        description="UC2 CANopen command-line tool — control motors, lasers, LEDs over CAN",
    )
    p.add_argument("--port", "-p", default=None, help="Waveshare serial port (auto-detected)")
    p.add_argument("--bitrate", "-b", type=int, default=500_000, help="CAN bitrate (default 500k)")

    sub = p.add_subparsers(dest="cmd", required=True)

    # scan
    s = sub.add_parser("scan", help="Scan for nodes on the CAN bus")
    s.add_argument("--timeout", type=float, default=3.0)

    # move
    s = sub.add_parser("move", help="Move a motor axis")
    s.add_argument("--node", "-n", type=int, default=NODE.MOT_X, help=f"Node ID (default {NODE.MOT_X})")
    s.add_argument("--axis", "-a", type=int, default=0, help="Axis index (default 0)")
    s.add_argument("--pos", type=int, required=True, help="Target position (steps)")
    s.add_argument("--speed", type=int, default=20000, help="Speed (steps/s, default 20000)")
    s.add_argument("--abs", dest="abs_", action="store_true", help="Absolute move (default: relative)")
    s.add_argument("--wait", "-w", action="store_true", help="Wait for motor to finish")
    s.add_argument("--timeout", type=float, default=30.0, help="Wait timeout (s)")

    # stop
    s = sub.add_parser("stop", help="Stop a motor axis")
    s.add_argument("--node", "-n", type=int, default=NODE.MOT_X)
    s.add_argument("--axis", "-a", type=int, default=0)

    # home
    s = sub.add_parser("home", help="Home a motor axis")
    s.add_argument("--node", "-n", type=int, default=NODE.MOT_X)
    s.add_argument("--axis", "-a", type=int, default=0)
    s.add_argument("--speed", type=int, default=15000)
    s.add_argument("--direction", type=int, default=-1, help="-1 or +1")
    s.add_argument("--wait", "-w", action="store_true")
    s.add_argument("--timeout", type=float, default=30.0)

    # laser
    s = sub.add_parser("laser", help="Set laser PWM")
    s.add_argument("--node", "-n", type=int, default=NODE.LASER_0)
    s.add_argument("--ch", type=int, default=0, help="Laser channel (0-3)")
    s.add_argument("--pwm", type=int, required=True, help="PWM value (0=off, up to 1023)")

    # led
    s = sub.add_parser("led", help="Set LED color")
    s.add_argument("--node", "-n", type=int, default=NODE.LED_0)
    s.add_argument("--r", type=int, default=0)
    s.add_argument("--g", type=int, default=0)
    s.add_argument("--b_val", dest="b", type=int, default=0)
    s.add_argument("--off", action="store_true", help="Turn LEDs off")

    # status
    s = sub.add_parser("status", help="Read node status")
    s.add_argument("--node", "-n", type=int, required=True)

    # sniff
    sub.add_parser("sniff", help="Dump raw CAN frames")

    # reboot
    s = sub.add_parser("reboot", help="Reboot a node")
    s.add_argument("--node", "-n", type=int, required=True)

    args = p.parse_args()

    handlers = {
        "scan": cmd_scan, "move": cmd_move, "stop": cmd_stop,
        "home": cmd_home, "laser": cmd_laser, "led": cmd_led,
        "status": cmd_status, "sniff": cmd_sniff, "reboot": cmd_reboot,
    }
    handlers[args.cmd](args)


if __name__ == "__main__":
    main()
