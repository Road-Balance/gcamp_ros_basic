#! /usr/bin/env python

import rospy

rospy.init_node("drive_forward")

rospy.loginfo("This is loginfo")
rospy.logdebug("This is logdebug")
rospy.logwarn("This is logwarn")
rospy.logerr("This is logerr")
rospy.logfatal("This is logfatal")

print("This is std print")