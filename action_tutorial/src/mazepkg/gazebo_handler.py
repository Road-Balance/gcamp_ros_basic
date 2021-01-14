#! /usr/bin/env python

"""
Fuctions for Gazebo model Spawn and Deletion

created by kimsooyoung : https://github.com/kimsooyoung
"""

import rospy
import rospkg

from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, DeleteModel


def GazeboSpawnModel(name, pkg_name, pose, namespace="", frame="world"):
    # model_name
    model_name = name

    # model_xml
    rospack = rospkg.RosPack()
    model_path = rospack.get_path(pkg_name) + "/models/"

    with open(model_path + model_name + ".urdf", "r") as xml_file:
        model_xml = xml_file.read().replace("\n", "")

    # robot_namespace
    robot_namespace = namespace

    # initial_pose
    initial_pose = Pose()
    initial_pose.position = pose.position

    # z rotation -pi/2 to Quaternion
    initial_pose.orientation = pose.orientation

    # reference_frame
    reference_frame = frame

    # service call
    spawn_model_prox = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
    result = spawn_model_prox(
        model_name, model_xml, robot_namespace, initial_pose, reference_frame
    )

    """ result fromat
    bool success
    string status_message
    """

    print(result.status_message)


def GazeboDeleteModel(name):

    """
    string model_name
    ---
    bool success
    string status_message

    """

    delete_srv = DeleteModel()

    model_name = name

    spawn_model_prox = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    result = spawn_model_prox(model_name)

    print(result.status_message)


def GazeboResetSimulation():
    spawn_model_prox = rospy.ServiceProxy("gazebo/reset_simulation", Empty)
    result = spawn_model_prox()

    rospy.loginfo("Gazebo Simulation Reset")


# rospy.init_node("gazebo_spawn_handler")


# TODO: Respawn robot model
# rosservice call /gazebo/set_model_state "model_state:
#   model_name: 'tinybot'
#   pose:
#     position:
#       x: 0.0
#       y: 0.0
#       z: 0.5
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: -0.707
#       w: 0.707
#   twist:
#     linear:
#       x: 0.0
#       y: 0.0
#       z: 0.0
#     angular:
#       x: 0.0
#       y: 0.0
#       z: 0.0
#   reference_frame: 'world'"
