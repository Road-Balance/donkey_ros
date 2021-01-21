#!/usr/bin/env python

import time
import rospy
from threading import Thread
from ackermann_msgs.msg import AckermannDriveStamped


class PCA9685:
    """
    PWM motor controler using PCA9685 boards.
    This is used for most RC Cars
    """

    def __init__(
        self, channel, address=0x40, frequency=60, busnum=None, init_delay=0.1
    ):

        self.default_freq = 60
        self.pwm_scale = frequency / self.default_freq

        import Adafruit_PCA9685

        # Initialise the PCA9685 using the default address (0x40).
        if busnum is not None:
            from Adafruit_GPIO import I2C

            # replace the get_bus function with our own
            def get_bus():
                return busnum

            I2C.get_default_bus = get_bus
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel
        time.sleep(init_delay)  # "Tamiya TBLE-02" makes a little leap otherwise

        self.pulse = 340
        self.prev_pulse = 340
        self.running = True

    def set_pwm(self, pulse):
        try:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))
        except:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))

    def run(self, pulse):
        pulse_diff = pulse - self.prev_pulse
        
        if (abs(pulse_diff) > 40):
            if pulse_diff > 0:
                pulse += (0.7 * pulse_diff)
            else:
                pulse -= (0.7 * pulse_diff)
        
        self.set_pwm(pulse)
        self.prev_pulse = pulse

    def set_pulse(self, pulse):
        self.pulse = pulse

    def update(self):
        while self.running:
            self.set_pulse(self.pulse)


class Vehicle(object):
    def __init__(self, name="donkey_ros"):

        self._throttle = PCA9685(channel=0, busnum=1)
        rospy.loginfo("Throttle Controler Awaked!!")

        self._steering_servo = PCA9685(channel=1, busnum=1)
        rospy.loginfo("Steering Controler Awaked!!")

        self._name = name
        self._teleop_sub = rospy.Subscriber(
            "/donkey_teleop", AckermannDriveStamped, self.joy_callback, queue_size=1, buff_size=2**24
        )
        rospy.loginfo("Teleop Subscriber Awaked!! Waiting for joystick...")

    def joy_callback(self, msg):
        speed_pulse = msg.drive.speed
        steering_pulse = msg.drive.steering_angle

        print('speed_pulse : ' + str(speed_pulse) + " / " + 'steering_pulse : ' + str(steering_pulse))

        self._throttle.run(speed_pulse)
        self._steering_servo.run(steering_pulse)


if __name__ == "__main__":

    rospy.init_node("donkey_control")
    myCar = Vehicle("donkey_ros")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
