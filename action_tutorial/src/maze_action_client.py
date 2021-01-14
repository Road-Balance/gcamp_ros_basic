#! /usr/bin/env python

"""
action client node for maze gazebo example

referenced from robotigniteacademy
url : https://get-help.robotigniteacademy.com/t/understanding-ros-actions-client-class-exercise/1498

list input part is referenced from geeksforgeeks
url : https://www.geeksforgeeks.org/python-get-a-list-as-input-from-user/
"""

import time
import rospy
import actionlib

from enum import IntEnum
from action_tutorial.msg import MazeAction, MazeGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# For more detail, search actionlib_msgs/GoalStatus
class ActionState(IntEnum):
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


action_server_name = "/maze_action_server"
rospy.init_node("maze_action_client")
action_client = actionlib.SimpleActionClient(action_server_name, MazeAction)

rospy.loginfo("Action Server Found..." + action_server_name)

goal = MazeGoal()
user_list = []


# try block to handle the exception
try:
    print("Enter numbers [or stop] : ")

    while True:
        user_list.append(int(input()))
# if the input is not-integer, just print the list
except:
    print("Your sequence list : ", user_list)

goal.turning_sequence = user_list

action_client.send_goal(goal, feedback_cb=fb_callback)
state_result = action_client.get_state()

rate = rospy.Rate(1)
rospy.loginfo("State Result from Server : " + str(state_result))

while state_result < ActionState.PREEMPTED:
    # Doing Stuff while waiting for the Server to give a result....
    rate.sleep()
    state_result = action_client.get_state()

if state_result == ActionState.SUCCEEDED:
    rospy.loginfo("Action Done State Result : " + str(state_result))
else:
    rospy.logwarn("Something went wrong, result state : " + str(state_result))
