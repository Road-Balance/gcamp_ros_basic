#! /usr/bin/env python

"""
referenced from programcreek

url : https://www.programcreek.com/python/example/93572/rospkg.RosPack
"""

import rospy
import rospkg
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel

rospy.init_node("gazebo_spawn_model")

# model_name
model_name = "r2d2"

# model_xml
rospack = rospkg.RosPack()
model_path = rospack.get_path("service_tutorial") + "/models/"

with open(model_path + model_name + ".urdf", "r") as xml_file:
    model_xml = xml_file.read().replace("\n", "")

# robot_namespace
robot_namespace = ""

# initial_pose
initial_pose = Pose()
initial_pose.position.x = -2
initial_pose.position.y = 1
initial_pose.position.z = 1

# z rotation -pi/2 to Quaternion
initial_pose.orientation.z = -0.707
initial_pose.orientation.w = 0.707

# reference_frame
reference_frame = "world"

# service call
spawn_model_prox = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
result = spawn_model_prox(
    model_name, model_xml, robot_namespace, initial_pose, reference_frame
)

""" result fromat
bool success
string status_message
"""

print(result)
