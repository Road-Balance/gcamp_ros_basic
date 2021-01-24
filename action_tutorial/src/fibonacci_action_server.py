#! /usr/bin/env python

"""
Fibonacci Action Server

referenced from wiki.ros.org

url: http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29#Compiling
"""

import rospy
import actionlib
from actionlib_tutorials.msg import FibonacciFeedback, FibonacciResult, FibonacciAction

g_feedback = FibonacciFeedback()
g_result = FibonacciResult()


def execute_cb(goal):
    r = rospy.Rate(5)
    success = True

    g_feedback.sequence = []
    g_feedback.sequence.append(0)
    g_feedback.sequence.append(1)

    rospy.loginfo(
        "Fibonacci Action Server Executing, creating fibonacci sequence of order %i with seeds %i, %i"
        % (goal.order, g_feedback.sequence[0], g_feedback.sequence[1])
    )

    for i in range(1, goal.order):
        if _as.is_preempt_requested():
            rospy.loginfo("The goal has been cancelled/preempted")
            _as.set_preempted()
            success = False
            break

        g_feedback.sequence.append(g_feedback.sequence[i] + g_feedback.sequence[i - 1])
        _as.publish_feedback(g_feedback)
        r.sleep()

    if success:
        g_result.sequence = g_feedback.sequence
        rospy.loginfo("Succeeded calculating the Fibonacci")
        _as.set_succeeded(g_result)


rospy.init_node("fibonacci")

_as = actionlib.SimpleActionServer(
    "fibonacci_action_server", FibonacciAction, execute_cb, False
)
_as.start()

print("==== Waiting for Client Goal...  ====")

rospy.spin()
