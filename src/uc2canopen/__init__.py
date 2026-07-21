"""
uc2canopen — Python CANopen master for openUC2 microscope hardware.

Quick start:
    from uc2canopen import UC2Client, NODE

    uc2 = UC2Client(port="/dev/ttyUSB0")
    uc2.motor.move(axis=0, position=1000, speed=20000, node_id=NODE.MOT_X)
    uc2.motor.wait_for_idle(axis=0, node_id=NODE.MOT_X)
    uc2.laser.set_value(channel=0, pwm=512, node_id=NODE.LASER_0)
    uc2.led.fill(r=255, g=0, b=0, node_id=NODE.LED_0)
    uc2.galvo.raster_scan(nx=128, ny=128, node_id=NODE.GALVO)
    uc2.close()
"""

import logging

# Library best practice: attach a NullHandler so log records are silently
# discarded unless the application configures a handler.
logging.getLogger(__name__).addHandler(logging.NullHandler())

from .client import (
    UC2Client,
    Motor,
    Laser,
    Led,
    Galvo,
    Pid,
    Encoder,
    DigitalIO,
    Analog,
    Collision,
    State,
)
from .od import OD, NODE
from .sdo import SdoClient, SdoError
from .waveshare_bus import WaveshareBus, find_waveshare_port
from .aio import (
    AsyncUC2Client,
    MotorUpdateEvent,
    MotorDoneEvent,
    HomingChangedEvent,
    HeartbeatEvent,
)

__all__ = [
    "AsyncUC2Client",
    "MotorUpdateEvent",
    "MotorDoneEvent",
    "HomingChangedEvent",
    "HeartbeatEvent",
    "UC2Client",
    "Motor",
    "Laser",
    "Led",
    "Galvo",
    "Pid",
    "Encoder",
    "DigitalIO",
    "Analog",
    "Collision",
    "State",
    "OD",
    "NODE",
    "SdoClient",
    "SdoError",
    "WaveshareBus",
    "find_waveshare_port",
]

__version__ = "0.2.0"
