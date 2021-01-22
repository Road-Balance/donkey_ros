#!/usr/bin/env python

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

rospy.init_node("webcam_pub", anonymous=True)
image_pub = rospy.Publisher("webcam_image", Image, queue_size=1)

bridge = CvBridge()

while not rospy.is_shutdown():
    # Capture frame-by-frame
    ret, cv_image = cap.read()

    # Display the resulting frame
    # cv2.imshow('frame',cv_image)
    # cv2.waitKey(3)
    image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
