#! /usr/bin/env python

"""
Fibonacci Action Client

referenced from wiki.ros.org

url: http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29 
"""

import rospy
import actionlib
from actionlib_tutorials.msg import FibonacciAction, FibonacciGoal

# Action Status (primitive)
PENDING = 0
ACTIVE = 1
PREEMPTED = 2
SUCCEEDED = 3
ABORTED = 4
REJECTED = 5
PREEMPTING = 6
RECALLING = 7
RECALLED = 8
LOST = 9


def fb_callback(feedback):
    print(feedback)


rospy.init_node("fibonacci_client")
rate = rospy.Rate(1)

client = actionlib.SimpleActionClient("fibonacci_action_server", FibonacciAction)
client.wait_for_server()

goal = FibonacciGoal()
goal.order = 20

print("==== Sending Goal to Server ====")
client.send_goal(goal, feedback_cb=fb_callback)

state_result = client.get_state()

while state_result < PREEMPTED:
    # Doing Stuff while waiting for the Server to give a result....
    rate.sleep()
    state_result = client.get_state()

if state_result == SUCCEEDED:
    rospy.loginfo("Action Done State Result : " + str(state_result))
else:
    rospy.logwarn("Something went wrong, result state : " + str(state_result))
