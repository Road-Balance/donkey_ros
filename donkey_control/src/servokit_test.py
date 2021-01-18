#!/usr/bin/python

import numpy as np
from adafruit_servokit import ServoKit
import board
import busio
import time

print("Initializing Servos")
_i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
_kit = ServoKit(channels=16, i2c=_i2c_bus0, address=0x40)
print("Done initializing")

_kit.servo[0].angle = 440
_kit.servo[1].angle = 300
