#! /usr/bin/env python

import rospy
import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (400,400), 10, 255)

    cv2.imshow("Image window", cv_image)

    # green
    # [  0 110   0]
    # gray
    # [105 105 105]

    print((cv_image[400, 400]))
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main():
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
