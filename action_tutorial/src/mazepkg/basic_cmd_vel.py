#! /usr/bin/env python

"""
action client node for maze gazebo example

created by kimsooyoung : https://github.com/kimsooyoung
"""

import rospy
from geometry_msgs.msg import Twist

GoForward = Twist()
GoForward.linear.x = 1.0

Stop = Twist()
Stop.linear.x = 0.0

Turn = Twist()
