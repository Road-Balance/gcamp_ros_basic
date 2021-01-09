#! /usr/bin/env python

import rospy
import actionlib
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from action_tutorial.msg import MazeAction, MazeFeedback, MazeResult

UP = 0
RIGHT = 1
DOWN = 2
LEFT = 3

class MazeAction(object):

    _feedback = MazeFeedback()
    _result = MazeResult()

    # control command => TODO: make header 
    _turning_time = 5
    _turning_vel = 0.78535

    _go_forward = Twist()
    _go_forward.linear.x = 1.0

    _stop = Twist()
    _stop.linear.x = 0.0

    _turn = Twist()
    _turn.angular.z = _turning_vel

    def __init__(self, name):
        print('MazeAction Constructor')
        self._action_name = name
        self._scan = []
        self._current_direction = 3 # 0 1 2 3

        self._publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self._subscriber = rospy.Subscriber("/scan", LaserScan, self.sb_calback )
        # self._action_server = actionlib.SimpleActionServer(self._action_name, MazeAction, execute_cb=self.ac_callback)
        # self._action_server.start()

        self._rate = rospy.Rate(5)

    @property
    def scan(self):
        return self._scan

    def robot_turn(self):
        pass

    def robot_go_forward(self):
        print('Robot Go Forward')

        self._rate.sleep()
        
        while self._scan[360] > 1.0:
            self._publisher.publish(self._go_forward)
        
        self._publisher.publish(self._stop)

    def sb_calback(self, data):
        self._scan = data.ranges

    def ac_callback(self, goal):
        success = True

        rospy.loginfo('==== Maze Action Server Executing ====')

        for i, val in enumerate(goal.turning_sequence):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
                

            self._rate.sleep()

        if success:
            self._result.success = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('maze_action')
    server = MazeAction(rospy.get_name())

    server.robot_go_forward()

    # while not rospy.is_shutdown():
    #     server.robot_go_forward()