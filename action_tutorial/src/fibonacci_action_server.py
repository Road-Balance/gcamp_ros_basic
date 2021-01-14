#! /usr/bin/env python

"""
example code for basic rospy action server

referenced from ros wiki
url : http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29
"""

#! /usr/bin/env python

import rospy
import actionlib
from actionlib_tutorials.msg import FibonacciAction, FibonacciFeedback, FibonacciResult


class FibonacciAction(object):

    _feedback = FibonacciFeedback()
    _result = FibonacciResult()

    def __init__(self, name):
        self._action_name = name
        self._action_server = actionlib.SimpleActionServer(
            self._action_name,
            FibonacciAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._action_server.start()

    def execute_cb(self, goal):
        rate = rospy.Rate(1)

        success = True

        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)

        rospy.loginfo(
            "%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i"
            % (
                self._action_name,
                goal.order,
                self._feedback.sequence[0],
                self._feedback.sequence[1],
            )
        )

        for i in range(1, goal.order):
            if self._action_server.is_preempt_requested():
                rospy.loginfo("%s: Preempted" % self._action_name)
                self._action_server.set_preempted()
                success = False
                break
            self._feedback.sequence.append(
                self._feedback.sequence[i] + self._feedback.sequence[i - 1]
            )
            self._action_server.publish_feedback(self._feedback)
            rate.sleep()

        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo("%s: Succeeded" % self._action_name)
            self._action_server.set_succeeded(self._result)


if __name__ == "__main__":
    rospy.init_node("fibonacci")
    server = FibonacciAction("fibonacci")
    rospy.spin()
