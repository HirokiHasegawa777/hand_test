#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
import sys
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def main():
    global completed
    completed = False
    arm_joint_trajectory_example = ArmJointTrajectoryExample()
    print("GO")
    if completed:
        pass
    else:
        arm_joint_trajectory_example.go()

if __name__ == "__main__":
    print("OK")
    rospy.init_node("main_move")
    main()
    rospy.spin()
