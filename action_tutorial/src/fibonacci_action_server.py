#! /usr/bin/env python
import rospy
import actionlib
from actionlib_tutorials.msg import FibonacciFeedback, FibonacciResult, FibonacciAction


class FibonacciActionClass(object):  # Be care for Naming

    _feedback = FibonacciFeedback()
    _result = FibonacciResult()

    def __init__(self):
        self._as = actionlib.SimpleActionServer(
            "fibonacci_action_server", FibonacciAction, self.goal_callback, False
        )
        self._as.start()

        print("==== Fibonacci Action Class Constructed ====")
        print("==== Waiting for Client Goal...  ====")

    def goal_callback(self, goal):
        r = rospy.Rate(1)
        success = True

        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)

        rospy.loginfo(
            "Fibonacci Action Server Executing, creating fibonacci sequence of order %i with seeds %i, %i"
            % (goal.order, self._feedback.sequence[0], self._feedback.sequence[1])
        )

        for i in range(1, goal.order):
            if self._as.is_preempt_requested():
                rospy.loginfo("The goal has been cancelled/preempted")
                self._as.set_preempted()
                success = False
                break

            self._feedback.sequence.append(
                self._feedback.sequence[i] + self._feedback.sequence[i - 1]
            )
            self._as.publish_feedback(self._feedback)
            r.sleep()

        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo("Succeeded calculating the Fibonacci")
            self._as.set_succeeded(self._result)


if __name__ == "__main__":
    rospy.init_node("fibonacci")
    FibonacciActionClass()
    rospy.spin()
