#!/usr/bin/env python

"""
Class for low level control of our car. It assumes ros-12cpwmboard has been
installed.

Listens to /dkcar/control/cmd_vel for corrective actions to the /cmd_vel coming from keyboard or joystick

"""
import rospy
from i2cpwm_board.msg import Servo, ServoArray
from geometry_msgs.msg import Twist
import time

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

        if abs(pulse_diff) > 40:
            if pulse_diff > 0:
                pulse += 0.7 * pulse_diff
            else:
                pulse -= 0.7 * pulse_diff

        self.set_pwm(pulse)
        self.prev_pulse = pulse

    def set_pulse(self, pulse):
        self.pulse = pulse

    def update(self):
        while self.running:
            self.set_pulse(self.pulse)

class ServoConvert:
    def __init__(self, id=1, center_value=370, range=90, direction=1):
        self.value = 0.0
        self.value_out = center_value
        self._center = center_value
        self._range = range
        self._half_range = 0.5 * range # 45
        self._dir = direction # 1 or -1 
        self.id = id

        # --- Convert its range in [-1, 1]
        self._sf = 1.0 / self._half_range # 1 / 45

    def get_value_out(self, value_in):
        # --- twist type value is in  [-1, 1]
        self.value = value_in
        self.value_out = int(self._dir * value_in * self._half_range + self._center)
        # print self.id, self.value_out
        return self.value_out


def saturate(value, min, max):
    if value <= min:
        return min
    elif value >= max:
        return max
    else:
        return value


class DkLowLevelCtrl:
    def __init__(self):
        rospy.loginfo("Setting Up the Node...")

        # --- Initialize the node
        rospy.init_node("dk_llc")

        self._throttle = PCA9685(channel=0, busnum=1)
        rospy.loginfo("Throttle Controler Awaked!!")

        self._steering_servo = PCA9685(channel=1, busnum=1)
        rospy.loginfo("Steering Controler Awaked!!")

        self.actuators = {}
        self.actuators["throttle"] = ServoConvert(
            id=1, center_value=350, range=80
        )
        self.actuators["steering"] = ServoConvert(
            id=2, center_value=375, range=80, direction=1
        )  # -- positive left
        rospy.loginfo("> Actuators corrrectly initialized")

        self._servo_msg = ServoArray()
        for i in range(2):
            self._servo_msg.servos.append(Servo())

        # --- Create the servo array publisher
        self.ros_pub_servo_array = rospy.Publisher(
            "/servos_absolute", ServoArray, queue_size=1
        )
        rospy.loginfo("> Publisher corrrectly initialized")

        # --- Create a debug publisher for resulting cmd_vel
        self.ros_pub_debug_command = rospy.Publisher(
            "/dkcar/debug/cmd_vel", Twist, queue_size=1
        )
        rospy.loginfo("> Publisher corrrectly initialized")

        # --- Create the Subscriber to Twist commands
        self.ros_sub_twist = rospy.Subscriber(
            "/cmd_vel", Twist, self.update_message_from_command
        )
        rospy.loginfo("> Subscriber corrrectly initialized")

        # --- Create the Subscriber to obstacle_avoidance commands
        self.ros_sub_twist = rospy.Subscriber(
            "/dkcar/control/cmd_vel", Twist, self.update_message_from_chase
        )
        rospy.loginfo("> Subscriber corrrectly initialized")

        self.throttle_cmd = 0.0
        self.throttle_chase = 1.0
        self.steer_cmd = 0.0
        self.steer_chase = 0.0

        self._debud_command_msg = Twist()

        # --- Get the last time e got a commands
        self._last_time_cmd_rcv = time.time()
        self._last_time_chase_rcv = time.time()
        self._timeout_ctrl = 100
        self._timeout_blob = 1

        rospy.loginfo("Initialization complete")

    def update_message_from_command(self, message):
        self._last_time_cmd_rcv = time.time()
        self.throttle_cmd = message.linear.x
        self.steer_cmd = message.angular.z

    def update_message_from_chase(self, message):
        self._last_time_chase_rcv = time.time()
        self.throttle_chase = message.linear.x
        self.steer_chase = message.angular.z
        # print(self.throttle_chase, self.steer_chase)

    def compose_command_velocity(self):
        self.throttle = saturate(self.throttle_cmd * self.throttle_chase, -1, 1)

        # -- Add steering
        self.steer = saturate(self.steer_cmd + self.steer_chase, -1, 1)

        self._debud_command_msg.linear.x = self.throttle
        self._debud_command_msg.angular.z = self.steer

        self.ros_pub_debug_command.publish(self._debud_command_msg)

        self.set_actuators_from_cmdvel(self.throttle, self.steer)

    def set_actuators_from_cmdvel(self, throttle, steering):
        """
        Get a message from cmd_vel, assuming a maximum input of 1
        """
        # -- Convert vel into servo values
        self.actuators["throttle"].get_value_out(throttle)
        self.actuators["steering"].get_value_out(steering)
        # rospy.loginfo("Got a command v = %2.1f  s = %2.1f"%(throttle, steering))
        # self.send_servo_msg()

        self.set_pwm_pulse(self.actuators["throttle"].value_out, self.actuators["steering"].value_out)

        print( "throttle value : " + str(self.actuators["throttle"].value_out))
        print( "steering value : " + str(self.actuators["steering"].value_out))


    def set_pwm_pulse(self, speed_pulse, steering_pulse):
        self._throttle.run(speed_pulse)
        self._steering_servo.run(steering_pulse)


    def set_actuators_idle(self):
        # -- Convert vel into servo values
        self.throttle_cmd = 0.0
        self.steer_cmd = 0.0

    def reset_avoid(self):
        self.throttle_chase = 1.0
        self.steer_avoid = 0.0

    def send_servo_msg(self):
        for actuator_name, servo_obj in self.actuators.iteritems():
            self._servo_msg.servos[servo_obj.id - 1].servo = servo_obj.id
            self._servo_msg.servos[servo_obj.id - 1].value = servo_obj.value_out
            # rospy.loginfo("Sending to %s command %d"%(actuator_name, servo_obj.value_out))

        self.ros_pub_servo_array.publish(self._servo_msg)

    @property
    def is_controller_connected(self):
        # print time.time() - self._last_time_cmd_rcv
        return time.time() - self._last_time_cmd_rcv < self._timeout_ctrl

    @property
    def is_chase_connected(self):
        return time.time() - self._last_time_chase_rcv < self._timeout_blob

    def run(self):

        # --- Set the control rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.compose_command_velocity()

            # print self._last_time_cmd_rcv, self.is_controller_connected
            if not self.is_controller_connected:
                self.set_actuators_idle()

            if not self.is_chase_connected:
                self.reset_avoid()

            rate.sleep()


if __name__ == "__main__":
    dk_llc = DkLowLevelCtrl()
    dk_llc.run()