#! /usr/bin/env python

import time
import rospy
from geometry_msgs.msg import Twist

rospy.init_node("drive_forward")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
r = rospy.Rate(1)

forward = Twist()
stop = Twist()

forward.linear.x = 0.5
stop.linear.x = 0.0

start_time = time.time()

rospy.loginfo("==== Endless DriveForward node Started ====\n")


while not rospy.is_shutdown():
    pub.publish(forward)
