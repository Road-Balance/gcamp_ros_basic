#! /usr/bin/env python

import rospy
import rospy
import cv2

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class ImageConverter:
    def __init__(self):
        # self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=1)

        self._center_pixel = []
        self._bridge = CvBridge()
        self._image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # green
        # [  0 110   0]
        # gray
        # [105 105 105]

        self._center_pixel = cv_image[400, 400]

    @property
    def center_pixel(self):
        self._rate = rospy.Rate(5)
        self._rate.sleep()
        return self._center_pixel
