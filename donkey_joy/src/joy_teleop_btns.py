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

    default_pmw_speed = 390.0
    default_pmw_steering = 375.0
    throttle_scale = 1.0
    steer_scale = 6.0

    msg = AckermannDriveStamped()

    first_forward = True

    def __init__(self):
        self.reset_control()
        self.sub = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.pub = rospy.Publisher(
            "/donkey_teleop", AckermannDriveStamped, queue_size=1
        )
        rospy.loginfo("JoyTeleopBtn Constructed")

    def reset_control(self):
        self.msg.drive.steering_angle = self.default_pmw_steering
        self.msg.drive.speed = self.default_pmw_speed
        self.first_forward = True

    def reset_steer(self):
        self.msg.drive.steering_angle = self.default_pmw_steering

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

    def set_msg(self, steer, throttle, XABY_btn_val):

        self.msg.drive.steering_angle += steer * self.steer_scale
        self.msg.drive.speed += throttle * self.throttle_scale

        if throttle == 1 and self.first_forward == True:
            self.first_forward = False
            self.msg.drive.speed = 405

        # debugging
        # print(steer,throttle)
        # print(self.msg)

        X_btn = XABY_btn_val[0]
        A_btn = XABY_btn_val[1]

        if X_btn == 1:
            self.reset_steer()
        elif A_btn == 1:
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
