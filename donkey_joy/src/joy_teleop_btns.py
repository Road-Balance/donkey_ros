#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

""" ackermann_msgs/AckermannDriveStamped
---
header: 
  seq: 304
  stamp: 
    secs: 0
    nsecs: 0
  frame_id: ''
drive: 
  steering_angle: 375.0
  steering_angle_velocity: 0.0
  speed: 370.0
  acceleration: 0.0
  jerk: 0.0
---
"""


class JoyTeleopBtn(object):

    default_pmw_speed = 370.0
    default_pmw_steering = 375.0
    btn_scale = 10.0

    msg = AckermannDriveStamped()

    def __init__(self):
        self.reset_control()
        self.sub = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.pub = rospy.Publisher("donkey_teleop", AckermannDriveStamped, queue_size=1)
        rospy.loginfo("JoyTeleopBtn Constructed")

    def reset_control(self):
        self.msg.drive.steering_angle = self.default_pmw_steering
        self.msg.drive.speed = self.default_pmw_speed

    def joy_callback(self, data):
        try:
            steering_btn_val = data.axes[4]
            throttle_btn_val = data.axes[5]
            XABY_btn_val = data.buttons[:4]
        except Exception as e:
            print(e)
        finally:
            self.set_msg(steering_btn_val, throttle_btn_val, XABY_btn_val)
            self.pub_msg()

    def set_msg(self, steer, throttle, XABY):

        self.msg.drive.steering_angle += steer * self.btn_scale
        self.msg.drive.speed += throttle * self.btn_scale

        # print(steer,throttle)
        # print(self.msg)

        if XABY[0] == 1:
            self.reset_control()

    def pub_msg(self):
        self.pub.publish(self.msg)


if __name__ == "__main__":
    try:
        rospy.init_node("joy_teleop_btns")
        jt = JoyTeleopBtn()
        rospy.spin()
    except Exception as e:
        print(e)
    finally:
        rospy.loginfo("JoyTeleopBtn Example done...")
