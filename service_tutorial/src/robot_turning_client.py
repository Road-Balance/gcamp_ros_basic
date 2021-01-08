#! /usr/bin/env python

import sys
import rospy
from service_tutorial.srv import ControlTurningMessage, ControlTurningMessageRequest

rospy.init_node('robot_turning_client')
rospy.loginfo("==== Robot Turning Server Started ====")

rospy.wait_for_service('/control_robot_angle')
service_client = rospy.ServiceProxy('/control_robot_angle', ControlTurningMessage)

request_msg = ControlTurningMessageRequest()

while not rospy.is_shutdown():
    try:
        t = input('> Type turning time duration: ')
        vel = input('> Type turning angular velocity: ')

        if vel > 1.5707:    
            raise ArithmeticError("Velocity too high !!")

        request_msg.time_duration = t
        request_msg.angular_vel = vel
        break
    except ArithmeticError as e:
        rospy.logerr(e)
    except Exception as e:
        rospy.logerr("Not a number type number plz !!")

result = service_client(request_msg)

print(result)