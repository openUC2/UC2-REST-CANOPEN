# uc2canopen — Python CANopen master for openUC2

Control UC2 microscope motors, lasers, LEDs, and galvos directly over CAN bus
from Python — no ESP32 master required, no JSON-over-serial, just CANopen
SDO/PDO.

Two transports are supported:

- **MCP2515 SPI HAT (SocketCAN)** — recommended on a Raspberry Pi. The HAT
  enumerates as a native `can0` interface. *This is the default.*
- **[Waveshare USB-CAN-A](https://www.waveshare.com/wiki/USB-CAN-A)** — a USB
  dongle, handy on a laptop.

Same API shape as [UC2-REST](https://github.com/openUC2/UC2-REST), different
transport: CANopen instead of JSON-over-serial.

---

## 1. Raspberry Pi + MCP2515 HAT setup (one-time)

> Skip this section if you use the Waveshare USB adapter — see
> [Waveshare USB-CAN-A](#waveshare-usb-can-a-alternative) at the bottom.

The HAT is an **MCP2515** CAN controller on **SPI0** (CS = `SPI0_CE0`/GPIO8,
INT = GPIO12) with an **SN65HVD230** 3.3 V transceiver and a **12 MHz** crystal.

### 1a. Enable SPI and load the MCP2515 driver

Edit `/boot/firmware/config.txt` and add:

```ini
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=12,spimaxfrequency=10000000
```

- `oscillator=12000000` **must match the crystal on the board** (12 MHz here).
  A wrong value = `can0` comes up but never sees a frame.
- `interrupt=12` is the INT line wiring (GPIO12). If SPI is flaky, drop
  `spimaxfrequency` to `2000000`.

Reboot, then confirm the driver bound:

```bash
sudo reboot
# after it's back:
dmesg | grep -iE "mcp251|can0"
#   → mcp251x spi0.0 can0: MCP2515 successfully initialized.
```

### 1b. Bring the interface up @ 500 kbit/s

```bash
sudo ip link set can0 up type can bitrate 500000 restart-ms 100
ip -details link show can0          # state should be ERROR-ACTIVE
```

500 kbit/s is the UC2 firmware bitrate — it **must match** your nodes.
`restart-ms 100` auto-recovers from bus-off while debugging.

### 1c. Sanity-check the bus (optional but recommended)

```bash
sudo apt install -y can-utils
candump can0
```

You should see live traffic — CANopen heartbeats at `0x700 + node-ID`
(e.g. `0x70B` = node 11) and motor TPDOs at `0x180 + node-ID`. If frames
scroll, the whole chain works. Send an NMT "start all":

```bash
cansend can0 000#0100
```

### 1d. Make `can0` come up at boot (optional)

Create `/etc/systemd/system/can0.service`:

```ini
[Unit]
Description=Bring up can0 (MCP2515)
After=sys-subsystem-net-devices-can0.device
BindsTo=sys-subsystem-net-devices-can0.device

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/sbin/ip link set can0 up type can bitrate 500000 restart-ms 100
ExecStop=/sbin/ip link set can0 down

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable --now can0.service
```

---

## 2. Install the Python package

```bash
# with uv (recommended)
uv pip install -e .

# or with pip
pip install -e .
```

`python-can`'s SocketCAN backend is built in on Linux — nothing extra to
install for the HAT. (`pyserial` is pulled in too, only needed for the
Waveshare adapter.)

```bash
pip install UC2-REST-CANOPEN
```
---

## 3. Run the demo

With `can0` up (step 1b), from the repo root:

```bash
python src/motor_demo.py --motor-node 11
```

This scans for nodes, jogs motor node 11 back and forth, blinks the laser/LED
board, and prints node status. Useful flags: `--skip-laser --skip-led`,
`--steps`, `--speed`, `--channel can1`.

---

## 4. Python API

```python
from uc2canopen import UC2Client, NODE

# Defaults to SocketCAN can0 (the MCP2515 HAT) — bring it up first with
#   sudo ip link set can0 up type can bitrate 500000
uc2 = UC2Client()
# Other channel:        UC2Client(channel="can1")
# Waveshare USB adapter: UC2Client(port="/dev/ttyUSB0")

# Move motor X axis 1000 steps
uc2.motor.move(axis=0, position=1000, speed=20000, node_id=NODE.MOT_X)
uc2.motor.wait_for_idle(axis=0, node_id=NODE.MOT_X)
print(f"Motor at {uc2.motor.get_position(axis=0, node_id=NODE.MOT_X)} steps")

uc2.laser.set_value(channel=0, pwm=512, node_id=NODE.LASER_0)   # laser 50%
uc2.led.fill(r=255, g=0, b=0, node_id=NODE.LED_0)               # LEDs red
print(f"Uptime: {uc2.state.get_uptime(NODE.MOT_X)}s")

uc2.close()
```

## 5. CLI

The `uc2can` command also defaults to SocketCAN `can0`:

```bash
uc2can scan                          # find nodes on the bus
uc2can move --node 11 --pos 1000 --speed 20000 --wait
uc2can laser --node 20 --ch 0 --pwm 512
uc2can led --node 20 --r 255 --g 0 --b_val 0
uc2can status --node 11
uc2can sniff                         # dump raw CAN frames
uc2can reboot --node 11

# pick a transport/channel (global flags, before the subcommand):
uc2can --channel can1 scan
uc2can --interface waveshare --port /dev/ttyUSB0 scan
```

---

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
└── python-can BusABC   ── SocketCAN can0 (MCP2515 HAT)   ← default
                        └─ WaveshareBus (USB-CAN-A serial)
        │
        ▼ CAN bus @ 500 kbit/s
        │
    ┌───┴───┬───────┬────────┐
    │       │       │        │
  Slave   Slave   Slave    Slave
  node11  node12  node13   node20
  (mot X) (mot Y) (mot Z)  (illum)
```

The whole SDO/PDO stack is transport-agnostic (it talks to a python-can
`BusABC`), so the only difference between transports is which bus object
`UC2Client` builds.

## OD index reference

All indices are in `uc2canopen.od.OD` and match the firmware's
`UC2_OD_Indices.h` (generated from `uc2_canopen_registry.yaml`).

## Troubleshooting

| Symptom | Most likely cause |
|---|---|
| No `can0`; `dmesg` empty/error | SPI not enabled, wrong `interrupt=`, or SPI wiring |
| `can0` up but `candump` empty | Wrong `oscillator=` (crystal), bitrate ≠ 500 k, or missing 120 Ω termination |
| `RuntimeError: No Waveshare ... adapter found` | You asked for the Waveshare transport (`--port`/`interface="waveshare"`) but none is attached; for the HAT just use the default |
| `Failed to open CAN bus` on the HAT | `can0` isn't up — run `sudo ip link set can0 up type can bitrate 500000 restart-ms 100` |
| Error frames / bus-off | Bitrate mismatch, CANH↔CANL swapped, or no common GND |

## Waveshare USB-CAN-A (alternative)

No driver setup needed — plug it in and select it explicitly:

```bash
python src/motor_demo.py --interface waveshare --port /dev/ttyUSB0 --motor-node 11
uc2can --interface waveshare scan          # auto-detects the port
```

```python
uc2 = UC2Client(port="/dev/ttyUSB0")       # a port implies the Waveshare transport
```

## Requirements

- Python ≥ 3.10
- `python-can` ≥ 4.0 (SocketCAN backend built in on Linux)
- `pyserial` (only for the Waveshare adapter)
- Hardware: an MCP2515 SPI HAT **or** a Waveshare USB-CAN-A adapter, plus UC2 slave(s) on the bus

## License

MIT — same as the UC2-ESP32 firmware.
</content>
</invoke>
