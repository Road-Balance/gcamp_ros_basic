#! /usr/bin/env python

"""
basic rospy service example

referenced from wiki.ros.org

url : http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29 
"""

import sys
import rospy
from service_tutorial.srv import ControlTurningMessage, ControlTurningMessageRequest

rospy.init_node("robot_turning_client")
rospy.loginfo("==== Robot Turning Server Started ====")

rospy.wait_for_service("/control_robot_angle")
service_client = rospy.ServiceProxy("/control_robot_angle", ControlTurningMessage)

request_srv = ControlTurningMessageRequest()

while not rospy.is_shutdown():
    try:
        t = input("> Type turning time duration: ")
        vel = input("> Type turning angular velocity: ")

        if vel > 1.5707:
            raise ArithmeticError("Velocity too high !!")

        request_srv.time_duration = t
        request_srv.angular_vel = vel
        break
    except ArithmeticError as e:
        rospy.logerr(e)
    except Exception as e:
        rospy.logerr("Not a number type number plz !!")

result = service_client(request_srv)

print(result)
