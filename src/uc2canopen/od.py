"""
UC2 CANopen Object Dictionary indices and node IDs.

AUTO-DERIVED from the firmware's OD.h / uc2_canopen_registry.yaml.
These constants match the C-side UC2_OD_Indices.h exactly.

Usage:
    from uc2canopen.od import OD, NODE, SDO_TYPES
    node.sdo_write(OD.MOTOR_TARGET_POSITION, sub=1, value=1000, fmt="<i")
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Literal


# ============================================================================
# OD index constants — manufacturer area 0x2000+
# Mirror of UC2_OD_Indices.h / uc2_indices.py
# ============================================================================

class OD:
    """CANopen Object Dictionary indices for openUC2 satellite nodes."""

    # ── Motor (0x2000-0x200B) ──
    MOTOR_TARGET_POSITION   = 0x2000  # I32[4]  target in steps
    MOTOR_ACTUAL_POSITION   = 0x2001  # I32[4]  current position
    MOTOR_SPEED             = 0x2002  # U32[4]  steps/s
    MOTOR_COMMAND_WORD      = 0x2003  # U8      bit N = start axis N, bit N+4 = stop
    MOTOR_STATUS_WORD       = 0x2004  # U8[4]   bit 0 = running
    MOTOR_ENABLE            = 0x2005  # U8[4]   driver enable
    MOTOR_ACCELERATION      = 0x2006  # U32[4]  steps/s²
    MOTOR_IS_ABSOLUTE       = 0x2007  # U8[4]   0=relative, 1=absolute
    MOTOR_MIN_POSITION      = 0x2008  # I32[4]  soft limit min
    MOTOR_MAX_POSITION      = 0x2009  # I32[4]  soft limit max
    MOTOR_JERK              = 0x200A  # U32[4]
    MOTOR_IS_FOREVER        = 0x200B  # U8[4]   continuous move

    # ── Homing (0x2010-0x2016) ──
    HOMING_COMMAND          = 0x2010  # U8[4]   1=start
    HOMING_SPEED            = 0x2011  # U32[4]
    HOMING_DIRECTION        = 0x2012  # I8[4]   -1 or +1
    HOMING_TIMEOUT          = 0x2013  # U32[4]  ms
    HOMING_ENDSTOP_RELEASE  = 0x2014  # I32[4]  steps
    HOMING_ENDSTOP_POLARITY = 0x2015  # U8[4]
    HOMING_STATUS           = 0x2016  # U8[4]   0=idle,1=homing,2=done,3=timeout (TPDO1)

    # ── TMC2209 (0x2020-0x2027) ──
    TMC_MICROSTEPS             = 0x2020  # U16[4]
    TMC_RMS_CURRENT            = 0x2021  # U16[4]  mA
    TMC_STALLGUARD_THRESHOLD   = 0x2022  # U8[4]
    TMC_COOLSTEP_SEMIN         = 0x2023  # U8[4]
    TMC_COOLSTEP_SEMAX         = 0x2024  # U8[4]
    TMC_BLANK_TIME             = 0x2025  # U8[4]
    TMC_TOFF                   = 0x2026  # U8[4]
    TMC_STALL_COUNT            = 0x2027  # U32[4]

    # ── Hard limits (0x2030-0x2032) ──
    HARDLIMIT_COMMAND       = 0x2030  # U8[4]
    HARDLIMIT_ENABLED       = 0x2031  # U8[4]
    HARDLIMIT_POLARITY      = 0x2032  # U8[4]

    # ── Laser (0x2100-0x2106) ──
    LASER_PWM_VALUE          = 0x2100  # U16[4]
    LASER_MAX_VALUE          = 0x2101  # U16[4]
    LASER_PWM_FREQUENCY      = 0x2102  # U32[4]
    LASER_PWM_RESOLUTION     = 0x2103  # U8[4]
    LASER_DESPECKLE_PERIOD   = 0x2104  # U32[4]  us (0=off)
    LASER_DESPECKLE_AMPLITUDE = 0x2105  # U16[4]
    LASER_SAFETY_STATE       = 0x2106  # U8

    # ── LED (0x2200-0x2221) ──
    # Note: 0x2210 and 0x2211 are DOMAIN/segmented — not expedited SDO.
    LED_ARRAY_MODE          = 0x2200  # U8
    LED_BRIGHTNESS          = 0x2201  # U8
    LED_UNIFORM_COLOUR      = 0x2202  # U32  0x00RRGGBB
    LED_PIXEL_COUNT         = 0x2203  # U16
    LED_LAYOUT_WIDTH        = 0x2204  # U8
    LED_LAYOUT_HEIGHT       = 0x2205  # U8
    LED_PIXEL_DATA          = 0x2210  # DOMAIN (segmented transfer; not expedited)
    LED_SINGLE_PIXEL        = 0x2211  # 5 bytes: u16 idx + u8 r + u8 g + u8 b (DOMAIN)
    LED_SHAPE               = 0x2212  # 5 bytes: u8 shape + u8 radius + u8 r + u8 g + u8 b (DOMAIN)
    LED_PATTERN_ID          = 0x2220  # U8
    LED_PATTERN_SPEED       = 0x2221  # U16

    # ── Digital I/O (0x2300-0x2302) ──
    DIGITAL_INPUT_STATE         = 0x2300  # U8[8]
    DIGITAL_OUTPUT_COMMAND      = 0x2301  # U8[8]
    DIGITAL_INPUT_CHANGE_MASK   = 0x2302  # U8  (not in OD.h yet — registry-only)

    # ── Analog / DAC (0x2310-0x2320) ──
    ANALOG_INPUT_VALUE      = 0x2310  # U16[8]
    ANALOG_INPUT_FILTERED   = 0x2311  # U16[8]
    DAC_OUTPUT_VALUE        = 0x2320  # U16[4]

    # ── Collision detector (0x2330-0x2336) — GPIO slave, SDO-only ──
    COLLISION_REFERENCE     = 0x2330  # U16  MANUAL idle baseline (ADC counts)
    COLLISION_THRESHOLD     = 0x2331  # U16  MANUAL allowed deviation band
    COLLISION_SENSITIVITY   = 0x2332  # U8   consecutive out-of-band samples to trip
    COLLISION_COMMAND       = 0x2333  # U8   1=calibrate, 2=set AUTO, 3=set MANUAL
    COLLISION_MEAN          = 0x2334  # U16  rolling mean / adaptive baseline (ro)
    COLLISION_MODE          = 0x2335  # U8   0=AUTO (adaptive), 1=MANUAL
    COLLISION_SIGMA         = 0x2336  # U16  AUTO robust noise scale (ro)

    # ── Encoder (0x2340-0x2342) ──
    ENCODER_POSITION        = 0x2340  # I32[4]
    ENCODER_VELOCITY        = 0x2341  # I32[4]
    ENCODER_ZERO_OFFSET     = 0x2342  # I32[4]

    # ── Generic I2C passthrough (0x2350-0x2354) — GPIO slave, SDO-only ──
    # 0x2350/0x2353 are DOMAIN (40-byte buffers; segmented transfer, not expedited).
    I2C_COMMAND             = 0x2350  # DOMAIN[40]  [addr7, flags, rd_len, wr_len, delay_ms, wr...]
    I2C_TRIGGER             = 0x2351  # U8   write !=0 to execute; slave clears when done
    I2C_STATUS              = 0x2352  # U8   0 idle, 1 busy, 2 done-ok, 0x80|err = failure
    I2C_RESPONSE            = 0x2353  # DOMAIN[40]  bytes read back (ro)
    I2C_RESP_LEN            = 0x2354  # U8   number of valid bytes in I2C_RESPONSE

    # ── Joystick (0x2400-0x2403) ──
    JOYSTICK_AXIS              = 0x2400  # I16[4] (registry-only)
    JOYSTICK_BUTTONS           = 0x2401  # U16    (registry-only)
    JOYSTICK_SPEED_MULTIPLIER  = 0x2402  # U8     (registry-only)
    JOYSTICK_DEADZONE          = 0x2403  # U16    (registry-only)

    # ── System (0x2500-0x250A) ──
    FIRMWARE_VERSION        = 0x2500  # STRING[32]  (segmented read; not expedited)
    BOARD_NAME              = 0x2501  # STRING[32]  (segmented read; not expedited)
    ENABLED_MODULES         = 0x2502  # U32  bitmask of active modules
    UPTIME_SECONDS          = 0x2503  # U32
    FREE_HEAP_BYTES         = 0x2504  # U32
    CAN_ERROR_COUNTER       = 0x2505  # U32
    CPU_TEMPERATURE         = 0x2506  # I16  0.01 °C
    REBOOT_COMMAND          = 0x2507  # U8   write 1 to soft-reboot
    BUILD_TIMESTAMP         = 0x2508  # STRING[32]  __DATE__ __TIME__ (segmented read)
    MAC_ADDRESS             = 0x2509  # STRING[18]  AA:BB:CC:DD:EE:FF (segmented read)
    COMMANDED_NODE_ID       = 0x250A  # U8   write 1..127 to reassign this node's CAN id

    # ── Galvo / scanner (0x2600-0x260F) ──
    GALVO_TARGET_POSITION   = 0x2600  # I32[2]  sub 1=X, 2=Y
    GALVO_ACTUAL_POSITION   = 0x2601  # I32[2]
    GALVO_COMMAND_WORD      = 0x2602  # U8
    GALVO_STATUS_WORD       = 0x2603  # U8
    GALVO_SCAN_SPEED        = 0x2604  # U32
    GALVO_N_STEPS_LINE      = 0x2605  # U16
    GALVO_N_STEPS_PIXEL     = 0x2606  # U16
    GALVO_D_STEPS_LINE      = 0x2607  # U16
    GALVO_D_STEPS_PIXEL     = 0x2608  # U16
    GALVO_T_PRE_US          = 0x2609  # U16  pre-trigger settle (firmware OD.h is U16)
    GALVO_T_POST_US         = 0x260A  # U16  post-trigger hold (firmware OD.h is U16)
    GALVO_X_START           = 0x260B  # I32
    GALVO_Y_START           = 0x260C  # I32
    GALVO_X_STEP            = 0x260D  # I32
    GALVO_Y_STEP            = 0x260E  # I32
    GALVO_CAMERA_TRIGGER_MODE = 0x260F  # U8  0=off,1=per-pixel,2=per-line,3=per-frame
    GALVO_CAMERA_TRIGGER    = 0x260F  # deprecated alias for GALVO_CAMERA_TRIGGER_MODE
    GALVO_POINTS_DATA       = 0x2610  # DOMAIN  arbitrary scan point list (segmented)

    # ── PID (0x2700-0x2705) ──
    PID_SETPOINT            = 0x2700  # I32
    PID_ACTUAL_VALUE        = 0x2701  # I32
    PID_KP                  = 0x2702  # U32
    PID_KI                  = 0x2703  # U32
    PID_KD                  = 0x2704  # U32
    PID_ENABLE              = 0x2705  # U8

    # ── OTA (0x2F00-0x2F05) ──
    OTA_FIRMWARE_DATA       = 0x2F00  # DOMAIN
    OTA_FIRMWARE_SIZE       = 0x2F01  # U32
    OTA_FIRMWARE_CRC32      = 0x2F02  # U32
    OTA_STATUS              = 0x2F03  # U8
    OTA_BYTES_RECEIVED      = 0x2F04  # U32
    OTA_ERROR_CODE          = 0x2F05  # U8


class NODE:
    """Default CAN node-ID assignments for openUC2 bus.

    Aligned with firmware UC2_NODE namespace and uc2_canopen_registry.yaml.
    Names ending in `_0` are legacy aliases for hardware where the illumination
    board hosts both laser + LED on one node (combined config).
    """
    MASTER   = 1
    # Motors
    MOT_X    = 11
    MOT_Y    = 12
    MOT_Z    = 13
    MOT_A    = 14      # firmware: UC2_NODE::MOTOR_A = 14 (was 10 in legacy od.py)
    # Illumination — combined board hosts LED + laser on node 20 by default;
    # separate boards use LED=20, LASER=21.
    LED      = 20
    LED_0    = 20      # alias: combined-board LED + laser on node 20
    LASER    = 21
    LASER_0  = 20      # alias: combined-board laser on node 20
    LASER_1  = 21      # alias for separate-board laser on node 21
    JOYSTICK = 22      # firmware: UC2_NODE::JOYSTICK = 22
    # Scanners / feedback
    GALVO    = 30
    GALVO_2  = 31
    ENCODER  = 40
    PID      = 50


# ============================================================================
# SDO type helpers — maps OD index to struct format for pack/unpack
# ============================================================================

@dataclass
class ODEntry:
    """Metadata for one OD entry."""
    index: int
    name: str
    fmt: str        # struct format char: 'b','B','h','H','i','I'
    size: int       # byte size per element
    array: int      # 0 = scalar, N = array of N elements
    writable: bool

# Build the type table from the constants above + OD.h knowledge.
# Types track lib/uc2_od/OD.h (the firmware-side RAM layout). DOMAIN entries
# and the 5-byte led_single_pixel are excluded — they require segmented SDO.
_ENTRIES: list[ODEntry] = [
    # motor (0x2000-0x200B)
    ODEntry(0x2000, "motor_target_position",  "i", 4, 4, True),
    ODEntry(0x2001, "motor_actual_position",  "i", 4, 4, False),
    ODEntry(0x2002, "motor_speed",            "I", 4, 4, True),
    ODEntry(0x2003, "motor_command_word",     "B", 1, 0, True),
    ODEntry(0x2004, "motor_status_word",      "B", 1, 4, False),
    ODEntry(0x2005, "motor_enable",           "B", 1, 4, True),
    ODEntry(0x2006, "motor_acceleration",     "I", 4, 4, True),
    ODEntry(0x2007, "motor_is_absolute",      "B", 1, 4, True),
    ODEntry(0x2008, "motor_min_position",     "i", 4, 4, True),
    ODEntry(0x2009, "motor_max_position",     "i", 4, 4, True),
    ODEntry(0x200A, "motor_jerk",             "I", 4, 4, True),
    ODEntry(0x200B, "motor_is_forever",       "B", 1, 4, True),
    # homing (0x2010-0x2015)
    ODEntry(0x2010, "homing_command",         "B", 1, 4, True),
    ODEntry(0x2011, "homing_speed",           "I", 4, 4, True),
    ODEntry(0x2012, "homing_direction",       "b", 1, 4, True),
    ODEntry(0x2013, "homing_timeout",         "I", 4, 4, True),
    ODEntry(0x2014, "homing_endstop_release", "i", 4, 4, True),
    ODEntry(0x2015, "homing_endstop_polarity","B", 1, 4, True),
    ODEntry(0x2016, "homing_status",          "B", 1, 4, False),
    # tmc (0x2020-0x2027)
    ODEntry(0x2020, "tmc_microsteps",            "H", 2, 4, True),
    ODEntry(0x2021, "tmc_rms_current",           "H", 2, 4, True),
    ODEntry(0x2022, "tmc_stallguard_threshold",  "B", 1, 4, True),
    ODEntry(0x2023, "tmc_coolstep_semin",        "B", 1, 4, True),
    ODEntry(0x2024, "tmc_coolstep_semax",        "B", 1, 4, True),
    ODEntry(0x2025, "tmc_blank_time",            "B", 1, 4, True),
    ODEntry(0x2026, "tmc_toff",                  "B", 1, 4, True),
    ODEntry(0x2027, "tmc_stall_count",           "I", 4, 4, False),
    # hard limits (0x2030-0x2032)
    ODEntry(0x2030, "hardlimit_command",      "B", 1, 4, True),
    ODEntry(0x2031, "hardlimit_enabled",      "B", 1, 4, True),
    ODEntry(0x2032, "hardlimit_polarity",     "B", 1, 4, True),
    # laser (0x2100-0x2106)
    ODEntry(0x2100, "laser_pwm_value",          "H", 2, 4, True),
    ODEntry(0x2101, "laser_max_value",          "H", 2, 4, False),
    ODEntry(0x2102, "laser_pwm_frequency",      "I", 4, 4, True),
    ODEntry(0x2103, "laser_pwm_resolution",     "B", 1, 4, True),
    ODEntry(0x2104, "laser_despeckle_period",   "I", 4, 4, True),
    ODEntry(0x2105, "laser_despeckle_amplitude","H", 2, 4, True),
    ODEntry(0x2106, "laser_safety_state",       "B", 1, 0, False),
    # led (0x2200-0x2221); 0x2210/2211 are DOMAIN/multi-byte — handle separately
    ODEntry(0x2200, "led_array_mode",         "B", 1, 0, True),
    ODEntry(0x2201, "led_brightness",         "B", 1, 0, True),
    ODEntry(0x2202, "led_uniform_colour",     "I", 4, 0, True),
    ODEntry(0x2203, "led_pixel_count",        "H", 2, 0, False),
    ODEntry(0x2204, "led_layout_width",       "B", 1, 0, False),
    ODEntry(0x2205, "led_layout_height",      "B", 1, 0, False),
    ODEntry(0x2220, "led_pattern_id",         "B", 1, 0, True),
    ODEntry(0x2221, "led_pattern_speed",      "H", 2, 0, True),
    # digital I/O (0x2300-0x2302)
    ODEntry(0x2300, "digital_input_state",       "B", 1, 8, False),
    ODEntry(0x2301, "digital_output_command",    "B", 1, 8, True),
    ODEntry(0x2302, "digital_input_change_mask", "B", 1, 0, False),
    # analog / DAC (0x2310-0x2320)
    ODEntry(0x2310, "analog_input_value",     "H", 2, 8, False),
    ODEntry(0x2311, "analog_input_filtered",  "H", 2, 8, False),
    ODEntry(0x2320, "dac_output_value",       "H", 2, 4, True),
    # collision detector (0x2330-0x2336) — scalar, SDO-only
    ODEntry(0x2330, "collision_reference",    "H", 2, 0, True),
    ODEntry(0x2331, "collision_threshold",    "H", 2, 0, True),
    ODEntry(0x2332, "collision_sensitivity",  "B", 1, 0, True),
    ODEntry(0x2333, "collision_command",      "B", 1, 0, True),
    ODEntry(0x2334, "collision_mean",         "H", 2, 0, False),
    ODEntry(0x2335, "collision_mode",         "B", 1, 0, True),
    ODEntry(0x2336, "collision_sigma",        "H", 2, 0, False),
    # encoder (0x2340-0x2342)
    ODEntry(0x2340, "encoder_position",       "i", 4, 4, False),
    ODEntry(0x2341, "encoder_velocity",       "i", 4, 4, False),
    ODEntry(0x2342, "encoder_zero_offset",    "i", 4, 4, True),
    # I2C bridge (0x2351/0x2352/0x2354) — scalar handshake bytes (buffers are DOMAIN)
    ODEntry(0x2351, "i2c_trigger",            "B", 1, 0, True),
    ODEntry(0x2352, "i2c_status",             "B", 1, 0, False),
    ODEntry(0x2354, "i2c_resp_len",           "B", 1, 0, False),
    # joystick (0x2400-0x2403)
    ODEntry(0x2400, "joystick_axis",              "h", 2, 4, False),
    ODEntry(0x2401, "joystick_buttons",           "H", 2, 0, False),
    ODEntry(0x2402, "joystick_speed_multiplier",  "B", 1, 0, True),
    ODEntry(0x2403, "joystick_deadzone",          "H", 2, 0, True),
    # system (0x2502-0x250A)
    ODEntry(0x2502, "enabled_modules",        "I", 4, 0, False),
    ODEntry(0x2503, "uptime_seconds",         "I", 4, 0, False),
    ODEntry(0x2504, "free_heap_bytes",        "I", 4, 0, False),
    ODEntry(0x2505, "can_error_counter",      "I", 4, 0, False),
    ODEntry(0x2506, "cpu_temperature",        "h", 2, 0, False),
    ODEntry(0x2507, "reboot_command",         "B", 1, 0, True),
    ODEntry(0x250A, "commanded_node_id",      "B", 1, 0, True),
    # galvo (0x2600-0x260F)
    ODEntry(0x2600, "galvo_target_position",  "i", 4, 2, True),
    ODEntry(0x2601, "galvo_actual_position",  "i", 4, 2, False),
    ODEntry(0x2602, "galvo_command_word",     "B", 1, 0, True),
    ODEntry(0x2603, "galvo_status_word",      "B", 1, 0, False),
    ODEntry(0x2604, "galvo_scan_speed",       "I", 4, 0, True),
    ODEntry(0x2605, "galvo_n_steps_line",     "H", 2, 0, True),
    ODEntry(0x2606, "galvo_n_steps_pixel",    "H", 2, 0, True),
    ODEntry(0x2607, "galvo_d_steps_line",     "H", 2, 0, True),
    ODEntry(0x2608, "galvo_d_steps_pixel",    "H", 2, 0, True),
    ODEntry(0x2609, "galvo_t_pre_us",         "H", 2, 0, True),
    ODEntry(0x260A, "galvo_t_post_us",        "H", 2, 0, True),
    ODEntry(0x260B, "galvo_x_start",          "i", 4, 0, True),
    ODEntry(0x260C, "galvo_y_start",          "i", 4, 0, True),
    ODEntry(0x260D, "galvo_x_step",           "i", 4, 0, True),
    ODEntry(0x260E, "galvo_y_step",              "i", 4, 0, True),
    ODEntry(0x260F, "galvo_camera_trigger_mode", "B", 1, 0, True),
    # PID (0x2700-0x2705)
    ODEntry(0x2700, "pid_setpoint",           "i", 4, 0, True),
    ODEntry(0x2701, "pid_actual_value",       "i", 4, 0, False),
    ODEntry(0x2702, "pid_kp",                 "I", 4, 0, True),
    ODEntry(0x2703, "pid_ki",                 "I", 4, 0, True),
    ODEntry(0x2704, "pid_kd",                 "I", 4, 0, True),
    ODEntry(0x2705, "pid_enable",             "B", 1, 0, True),
]

SDO_TYPES: dict[int, ODEntry] = {e.index: e for e in _ENTRIES}
