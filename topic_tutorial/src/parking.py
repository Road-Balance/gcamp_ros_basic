#! /usr/bin/env python

"""
Parking Assignment Answer

Try it out!!
"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def callback(data):
    laser_range = data.ranges
    cmd_vel = Twist()

    if laser_range[360] > 0.8:
        cmd_vel.linear.x = 1.0
    else:
        cmd_vel.linear.x = 0.0
        pub.publish(cmd_vel)

        sub.unregister()
        rospy.signal_shutdown("Parking Done")

    pub.publish(cmd_vel)


rospy.init_node("parking", disable_signals=True)
rospy.loginfo("==== parking node Started ====\n")

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
sub = rospy.Subscriber("/scan", LaserScan, callback)

rospy.spin()
