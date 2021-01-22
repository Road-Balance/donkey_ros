#!/usr/bin/env python

import numpy as np
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

cap = cv2.VideoCapture(0)

rospy.init_node("image_converter", anonymous=True)

image_pub = rospy.Publisher("image_topic_2", Image, queue_size=1)
bridge = CvBridge()

rate = rospy.Rate(5)

while not rospy.is_shutdown():
    # Capture frame-by-frame
    ret, cv_image = cap.read()

    # Display the resulting frame
    # cv2.imshow('frame',cv_image)
    # cv2.waitKey(3)
    image_pub.publish(bridge.cv2_to_imgmsg(cv_image))
    
    rate.sleep()

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()