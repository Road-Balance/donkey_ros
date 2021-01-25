#!/usr/bin/python

import numpy as np

# from adafruit_servokit import ServoKit
from adafruit_pca9685 import PCA9685

import board
import busio
import time

print("Initializing Servos")
_i2c_bus1 = busio.I2C(board.SCL, board.SDA)
# _kit = PCA9685(channels=16, i2c=_i2c_bus1, address=0x40)
pca = PCA9685(_i2c_bus1)
pca.frequency = 60
print("Done initializing")

pca.channels[0].duty_cycle = 440
pca.channels[1].duty_cycle = 300


# _kit.servo[0].angle = 440
# _kit.servo[1].angle = 300
