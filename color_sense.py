#!/usr/bin/env pybricks-micropython

import logging
from time   import sleep
from random import randint

from ev3dev2.motor import OUTPUT_A, OUTPUT_D, OUTPUT_B, OUTPUT_C, LargeMotor, SpeedPercent, MoveTank
from ev3dev2.sensor.lego import InfraredSensor, ColorSensor
from ev3dev2.button import Button
from ev3dev2.led import Leds

#SenseColorOnScreen
# Change level to logging.DEBUG for more details
logging.basicConfig(level=logging.INFO,
                format="%(asctime)s %(levelname)5s %(filename)s: %(message)s")
log = logging.getLogger(__name__)

log.info("Starting GRIPP3R")
color_sensor = ColorSensor()
log.info("testing")

print("test")
log.error("damn it")
