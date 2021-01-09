#! /usr/bin/env python

import math
import time
import rospy
import actionlib

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from action_tutorial.msg import MazeAction, MazeFeedback, MazeResult
from tf.transformations import euler_from_quaternion, quaternion_from_euler

UP = 0
RIGHT = 1
DOWN = 2
LEFT = 3

class MazeAction(object):

    _feedback = MazeFeedback()
    _result = MazeResult()

    # control command => TODO: make header 
    _turning_time = 3.6
    _turning_vel = 0.392675

    _go_forward = Twist()
    _go_forward.linear.x = 1.0

    _stop = Twist()
    _stop.linear.x = 0.0

    _turn_cmd = Twist()

    def __init__(self, name):
        print('MazeAction Constructor')
        self._action_name = name
        self._yaw = 0.0
        self._scan = []
        self._current_direction = 3 # 0 1 2 3

        self._cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self._odom_sub = rospy.Subscriber ('/odom', Odometry, self.odom_callback)
        self._scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_calback )
        # self._action_server = actionlib.SimpleActionServer(self._action_name, MazeAction, execute_cb=self.ac_callback)
        # self._action_server.start()

        self._rate = rospy.Rate(5)

    def scan_calback(self, data):
        self._scan = data.ranges

    def odom_callback (self, data):
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self._yaw = euler_from_quaternion(orientation_list)

    def robot_go_forward(self):
        self._rate.sleep()
        
        while self._scan[360] > 1.0:
            self._cmd_pub.publish(self._go_forward)        
        self._cmd_pub.publish(self._stop)

    def robot_turn_direction(self, direction_in):
        direction_offset = direction_in - self._current_direction
        turn_sign = (-1 if direction_offset < 0 else 1)

        direction_offset *= turn_sign
        self._turn_cmd.angular.z = self._turning_vel * turn_sign

        print(direction_offset, turn_sign, self._turn_cmd)

        for i in range(direction_offset):
            start_time = time.time()
            while time.time() - start_time < self._turning_time:
                self._cmd_pub.publish(self._turn_cmd)
            self._cmd_pub.publish(self._stop)

        self._current_direction = direction_in

    def robot_turn_euler(self, euler_angle):
        target_rad = euler_angle * math.pi / 180

        print('target_rad : ', target_rad)

        turn_offset = 100
        self._rate.sleep()

        while abs(turn_offset) > 0.005:
            turn_offset = 0.7 * (target_rad - self._yaw)

            self._turn_cmd.angular.z = turn_offset
            self._cmd_pub.publish(self._turn_cmd)
            
            print("yaw / turn_offset : ", self._yaw, turn_offset)
        
        self._cmd_pub.publish(self._stop)

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

    @property
    def scan(self):
        return self._scan

    @property
    def yaw(self):
        return self._yaw

if __name__ == '__main__':
    rospy.init_node('maze_action')
    server = MazeAction(rospy.get_name())

    server.robot_turn(0)

    # while not rospy.is_shutdown():
    #     server.robot_go_forward()