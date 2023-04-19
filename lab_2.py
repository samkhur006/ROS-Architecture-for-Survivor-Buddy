#!/usr/bin/env python3

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage, Image
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import TwistStamped


import audio
import video
import globals
# Python 2/3 compatibility imports
import sys
import copy
import math
import rospy
import geometry_msgs.msg
import moveit_commander

import numpy as np


try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String

twist = TwistStamped()


class GenericBehavior(object):
    """
    Generic behavior class.
    """
    def __init__(self):
        self.pub = rospy.Publisher(
            "/move_group/display_planned_path", DisplayTrajectory, queue_size=20
        )
        globals.initialize()
        print("early print ",globals.count)
        self.sampub = rospy.Publisher('/sb_cmd_state', TwistStamped, queue_size=10)
        self.audio_sub = rospy.Subscriber("/audio", Float32MultiArray, callback=audio.callback_1, queue_size=1)
        self.camera_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, callback=self.callback_2, queue_size=1)
        rospy.loginfo("Node started.")
        audio.printing(6)
        print("new count is ",globals.count)
        #self.robot = moveit_commander.RobotCommander()
        #self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "survivor_buddy_head"
        #self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

    # execute_behaviour

    

 
if __name__ == '__main__':
    rospy.init_node("lab_1_node")
    moveit_commander.roscpp_initialize(sys.argv)
    ##############################
    # YOUR CODE HERE             #
    # Call the behavior(s)       #
    ##############################
    GenericBehavior()
    rospy.spin()
