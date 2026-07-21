# Changelog

All notable changes to `uc2canopen` are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/), and this project adheres to
semantic versioning.

## [0.2.0] — 2026-07-15

Sync with the openUC2 ESP32 firmware CANopen interface (`uc2_canopen_registry.yaml`
/ `UC2_OD_Indices.h`). This release adds high-level controllers for every device
class the firmware now exposes and brings the Object Dictionary in `od.py` up to
date with the current registry.

### Added — new device controllers on `UC2Client`

- **`uc2.galvo`** (`Galvo`) — galvo mirror scanner (OD `0x2600`):
  - `set_position(x, y)` / `goto(...)` — absolute XY move (DAC counts) + `GOTO`.
  - `raster_scan(...)` — full XY raster; mirrors the firmware
    `DeviceRouter::handleGalvoAct` config path, including the reused transport
    slots (`0x2607` → bidirectional, `0x2608` → line-settle samples) and the
    `x_step`/`y_step = (max − min) // n` computation.
  - `line_scan(...)` — single X-sweep line scan.
  - `stop()`, `estop()`, `get_position()`, `get_status()` (decoded flag dict).
  - Command-word (`CMD_*`) and camera-trigger (`TRIGGER_*`) constants.
- **`uc2.pid`** (`Pid`) — generic PID loop (OD `0x2700`): `set_setpoint`,
  `set_gains(kp, ki, kd)`, `set_enabled`, `get_actual`.
- **`uc2.encoder`** (`Encoder`) — quadrature/AS5600 feedback (OD `0x2340`):
  `get_position`, `get_velocity`, `set_zero`.
- **`uc2.digital`** (`DigitalIO`) — digital inputs/outputs (OD `0x2300`):
  `read_input`, `write_output`.
- **`uc2.analog`** (`Analog`) — ADC in / DAC out (OD `0x2310`/`0x2320`):
  `read_input`, `read_filtered`, `write_dac`.
- **`uc2.collision`** (`Collision`) — baseline-deviation collision detector on
  the GPIO slave (OD `0x2330`): `configure`, `calibrate`, `get_mean`,
  `get_sigma`, with `MODE_AUTO` / `MODE_MANUAL`.

### Added — extensions to existing controllers

- `Motor`: `get_homing_status` / `wait_for_homed` (via new `HOMING_STATUS`
  `0x2016`, also broadcast on TPDO1 byte 5), `set_soft_limits`,
  `set_acceleration`, `configure_tmc`, `set_hardlimit`, `clear_hardlimit`, plus
  `HOME_*` status constants.
- `Laser`: `get_max_value`, `set_pwm_frequency`, `set_pwm_resolution`,
  `set_despeckle` (new `LASER_DESPECKLE_PERIOD` `0x2104` / `_AMPLITUDE` `0x2105`),
  `get_safety_state`.
- `Led`: `set_mode`, `set_pattern`, `get_pixel_count`, plus `MODE_*` and
  `PATTERN_*` constants.
- `State`: `get_cpu_temperature` (`0x2506`), `get_can_errors`,
  `get_enabled_modules` (`0x2502`), and `set_node_id` (node provisioning via
  `COMMANDED_NODE_ID` `0x250A`).

### Added — Object Dictionary (`od.py`)

New index constants and expedited-SDO type entries mirroring the firmware:

- Homing: `HOMING_STATUS` (`0x2016`).
- Laser: `LASER_DESPECKLE_PERIOD` (`0x2104`), `LASER_DESPECKLE_AMPLITUDE` (`0x2105`).
- LED: `LED_LAYOUT_WIDTH` (`0x2204`), `LED_LAYOUT_HEIGHT` (`0x2205`),
  `LED_SHAPE` (`0x2212`, DOMAIN).
- Collision: `COLLISION_REFERENCE`…`COLLISION_SIGMA` (`0x2330`–`0x2336`).
- I2C bridge: `I2C_COMMAND`…`I2C_RESP_LEN` (`0x2350`–`0x2354`; buffers are DOMAIN).
- System: `BUILD_TIMESTAMP` (`0x2508`), `MAC_ADDRESS` (`0x2509`),
  `COMMANDED_NODE_ID` (`0x250A`); `CPU_TEMPERATURE` (`0x2506`) is now a real
  packable entry.
- Galvo: `GALVO_POINTS_DATA` (`0x2610`, DOMAIN).
- Also added expedited type-table entries that were previously index-only:
  encoder velocity/zero-offset, DAC output, analog filtered, digital
  change-mask, joystick, `ENABLED_MODULES`, and the full PID block.

### Changed

- `GALVO_CAMERA_TRIGGER` (`0x260F`) renamed to **`GALVO_CAMERA_TRIGGER_MODE`** to
  match the firmware. The old name remains as a deprecated alias.
- TPDO1 listener now parses the optional 6th byte (per-axis homing status) and
  stores it on `MotorState.homing_status`; older 5-byte frames still work.
- Package `__version__` corrected to track the packaged version (was stuck at
  `0.1.0`); now `0.2.0`.
- New controller classes are exported from the top-level package
  (`from uc2canopen import Galvo, Pid, Encoder, …`).

### Notes / limitations

- DOMAIN entries (arbitrary galvo point lists `0x2610`, LED pixel/shape data,
  raw-I2C buffers `0x2350`/`0x2353`, OTA) and STRING reads (`0x2500`, `0x2508`,
  `0x2509`) require **segmented/block SDO transfer**, which the current
  expedited-only `SdoClient` does not implement. Their index constants are
  provided; the transport is a follow-up.
- `tests/test_new_features.py` validates the exact SDO traffic for every new
  command against the firmware mapping, with no hardware required. An optional
  live smoke test runs with `UC2_LIVE=1`.

## [0.1.4] and earlier

Initial motor / laser / led / state controllers over CANopen SDO, with SocketCAN
(MCP2515 HAT) and Waveshare USB-CAN-A transports. See git history for details.
