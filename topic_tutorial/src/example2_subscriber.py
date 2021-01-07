#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan 

case = 'spin_mode'

def callback(data):
    # data.ranges
    print(data)

rospy.loginfo("==== Laser Scan Subscriber node Started, move forward during 10 seconds ====\n")

rospy.init_node('laser_scan')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
