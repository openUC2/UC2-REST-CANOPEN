# uc2canopen — Python CANopen master for openUC2

Control UC2 microscope motors, lasers, LEDs, and galvos directly over CAN bus
from Python — no ESP32 master required, no JSON-over-serial, just a
[Waveshare USB-CAN-A](https://www.waveshare.com/wiki/USB-CAN-A) adapter
plugged into your laptop or Raspberry Pi.

Same API shape as [UC2-REST](https://github.com/openUC2/UC2-REST), different
transport: CANopen SDO/PDO instead of JSON-over-serial.

## Quick start

```bash
# Install with uv (recommended)
uv pip install -e .

# Or with pip
pip install -e .
```

## Python API

```python
from uc2canopen import UC2Client, NODE

# Auto-detects the Waveshare adapter
uc2 = UC2Client()

# Move motor X axis 1000 steps
uc2.motor.move(axis=0, position=1000, speed=20000, node_id=NODE.MOT_X)
uc2.motor.wait_for_idle(axis=0, node_id=NODE.MOT_X)
pos = uc2.motor.get_position(axis=0, node_id=NODE.MOT_X)
print(f"Motor at {pos} steps")

# Laser on at 50%
uc2.laser.set_value(channel=0, pwm=512, node_id=NODE.LASER_0)

# LED fill red
uc2.led.fill(r=255, g=0, b=0, node_id=NODE.LED_0)

# System info
print(f"Uptime: {uc2.state.get_uptime(NODE.MOT_X)}s")

uc2.close()
```

## CLI

```bash
# Scan for nodes
uc2can scan

# Move motor
uc2can move --node 11 --pos 1000 --speed 20000 --wait

# Set laser
uc2can laser --node 20 --ch 0 --pwm 512

# Set LED
uc2can led --node 20 --r 255 --g 0 --b 0

# Read status
uc2can status --node 11

# Sniff CAN bus
uc2can sniff

# Reboot a node
uc2can reboot --node 11
```

## Architecture

```
Your Python script
    │
    ▼
UC2Client
├── motor  → Motor()    SDO writes to 0x2000-0x200B
├── laser  → Laser()    SDO writes to 0x2100
├── led    → Led()      SDO writes to 0x2200
├── state  → State()    SDO reads from 0x2500+
│
├── SdoClient           raw SDO upload/download over python-can
├── TpdoListener        background thread for motor state (TPDO1)
│
└── WaveshareBus        Waveshare USB-CAN-A serial protocol → python-can
        │
        ▼
    USB-CAN-A adapter
        │
        ▼ CAN bus @ 500 kbit/s
        │
    ┌───┴───┬───────┬────────┐
    │       │       │        │
  Slave   Slave   Slave    Slave
  node10  node11  node12   node20
  (mot A) (mot X) (mot Y)  (laser)
```

## Node-ID assignments

| Role | Node-ID | Python constant |
|------|---------|-----------------|
| Master (this script) | 1 | `NODE.MASTER` |
| Motor X | 11 | `NODE.MOT_X` |
| Motor Y | 12 | `NODE.MOT_Y` |
| Motor Z | 13 | `NODE.MOT_Z` |
| Motor A | 14 | `NODE.MOT_A` |
| LED / combined illum. board | 20 | `NODE.LED` (alias: `LED_0`, `LASER_0`) |
| Laser (separate board) | 21 | `NODE.LASER` (alias: `LASER_1`) |
| Joystick | 22 | `NODE.JOYSTICK` |
| Galvo | 30 | `NODE.GALVO` |
| Galvo 2 | 31 | `NODE.GALVO_2` |
| Encoder | 40 | `NODE.ENCODER` |
| PID | 50 | `NODE.PID` |

## OD index reference

All indices are in `uc2canopen.od.OD` and match the firmware's
`UC2_OD_Indices.h` (generated from `uc2_canopen_registry.yaml`).

## Requirements

- Python ≥ 3.10
- `pyserial` (for Waveshare adapter)
- `python-can` (CAN bus abstraction)
- Hardware: Waveshare USB-CAN-A adapter + UC2 slave(s) on the bus

## License

MIT — same as the UC2-ESP32 firmware.
