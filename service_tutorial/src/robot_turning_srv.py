#! /usr/bin/env python

import time
import rospy
from service_tutorial.srv import ControlTurningMessage, ControlTurningMessageResponse
from geometry_msgs.msg import Twist


def callback(request):

    print(request)

    # publish to cmd_vel
    cmd_vel = Twist()
    cmd_vel.angular.z = request.angular_vel
    start_time = time.time()

    print("\nRobot Turning...")
    while time.time() - start_time < request.time_duration:
        velocity_publisher.publish(cmd_vel)

    print("Done ...")
    cmd_vel.angular.z = 0.0
    velocity_publisher.publish(cmd_vel)

    response = ControlTurningMessageResponse()
    response.success = True

    # code for class necessity
    # if call_count > 3:
    #     rospy.logwarn("==== Service Called too many times, Shutdown ====")
    #     service_server.shutdown()

    return response


rospy.init_node("robot_turning_server")
rospy.loginfo("==== Robot Turning Server Started ====")

service_server = rospy.Service("/control_robot_angle", ControlTurningMessage, callback)
velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

rospy.spin()
