#! /usr/bin/env python

"""
action client node for maze gazebo example

created by kimsooyoung : https://github.com/kimsooyoung
"""


import math
import time
import rospy
import actionlib

from mazepkg.basic_cmd_vel import GoForward, Stop, Turn
from mazepkg.gazebo_handler import GazeboResetSimulation
from mazepkg.image_converter import ImageConverter

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

from action_tutorial.msg import MazeAction, MazeFeedback, MazeResult

direction_dict = { 0: -90, 1: 180, 2: 90, 3: 0 }
direction_str_dict = { 0: 'Up', 1: 'Right', 2: 'Down', 3: 'Left' }

class MazeActionClass(object):

    _feedback = MazeFeedback()
    _result = MazeResult()

    def __init__(self, name):
        self._action_name = name
        self._yaw = 0.0
        self._scan = []
        self._current_direction = 3 # 0 1 2 3

        self._cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self._odom_sub = rospy.Subscriber ('/odom', Odometry, self.odom_callback)
        self._scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_calback )
        # self._image_sub = rospy.Subscriber('')
        self._action_server = actionlib.SimpleActionServer(self._action_name, MazeAction, execute_cb=self.ac_callback, auto_start = False)
        self._action_server.start()

        self._rate = rospy.Rate(5)

        print('==== MazeActionClass Constructed ====')
        print('==== Waiting for Client Goal...  ====')

    def scan_calback(self, data):
        self._scan = data.ranges

    def odom_callback (self, data):
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self._yaw = euler_from_quaternion(orientation_list)

    def robot_go_forward(self):
        self._rate.sleep()
        
        while self._scan[360] > 0.9:
            self._cmd_pub.publish(GoForward)     
        self._cmd_pub.publish(Stop)

    def robot_turn(self, euler_angle):
        target_rad = euler_angle * math.pi / 180

        turn_offset = 100
        self._rate.sleep()

        while abs(turn_offset) > 0.005:
            turn_offset = 0.7 * (target_rad - self._yaw)

            Turn.angular.z = turn_offset
            self._cmd_pub.publish(Turn)
            
        self._cmd_pub.publish(Stop)

    def ac_callback(self, goal):
        success = True
        print('==== Maze Action Server Executing ====')

        for i, val in enumerate(goal.turning_sequence):
            # check that preempt has not been requested by the client
            if self._action_server.is_preempt_requested():
                rospy.logwarn('%s: Preempted' % self._action_name)
                self._action_server.set_preempted()
                success = False
                break
            
            self._feedback.feedback_msg = "Turning to " + direction_str_dict[val]
            self._action_server.publish_feedback(self._feedback)

            print('Turning Sequence : ' +  str(val))
            self.robot_turn(direction_dict[val])

            self._feedback.feedback_msg = "Moving Forward ..."
            self._action_server.publish_feedback(self._feedback)
            self.robot_go_forward()

            self._rate.sleep()

        # TODO: success condition => goal sign
        if success:
            ic = ImageConverter()
            center_pixel =  ic.center_pixel

            if sum(center_pixel) < 300 and center_pixel[1] > 100:
                self._result.success = True
                rospy.loginfo('Maze Escape Succeeded')
            else:
                self._result.success = False
                rospy.logerr('Maze Escape Failed')
                GazeboResetSimulation()

            self._action_server.set_succeeded(self._result)

    @property
    def scan(self):
        return self._scan

    @property
    def yaw(self):
        return self._yaw

if __name__ == '__main__':
    rospy.init_node('maze_action_server')
    server = MazeActionClass('maze_action_server')
    rospy.spin()