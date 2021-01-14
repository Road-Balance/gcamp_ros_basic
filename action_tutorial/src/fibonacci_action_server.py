#! /usr/bin/env python

"""
Fibonacci Action Server

referenced from wiki.ros.org

url: http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29#Compiling
"""

import rospy
import actionlib
from actionlib_tutorials.msg import FibonacciFeedback, FibonacciResult, FibonacciAction

_feedback = FibonacciFeedback()
_result = FibonacciResult()


def goal_callback(goal):
    r = rospy.Rate(1)
    success = True

    _feedback.sequence = []
    _feedback.sequence.append(0)
    _feedback.sequence.append(1)

    rospy.loginfo(
        "Fibonacci Action Server Executing, creating fibonacci sequence of order %i with seeds %i, %i"
        % (goal.order, _feedback.sequence[0], _feedback.sequence[1])
    )

    for i in range(1, goal.order):
        if _as.is_preempt_requested():
            rospy.loginfo("The goal has been cancelled/preempted")
            _as.set_preempted()
            success = False
            break

        _feedback.sequence.append(_feedback.sequence[i] + _feedback.sequence[i - 1])
        _as.publish_feedback(_feedback)
        r.sleep()

    if success:
        _result.sequence = _feedback.sequence
        rospy.loginfo("Succeeded calculating the Fibonacci")
        _as.set_succeeded(_result)


if __name__ == "__main__":
    rospy.init_node("fibonacci")

    _as = actionlib.SimpleActionServer(
        "fibonacci_action_server", FibonacciAction, goal_callback, False
    )
    _as.start()

    print("==== Waiting for Client Goal...  ====")

    rospy.spin()
