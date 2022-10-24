#! /usr/bin/env python
# -*- coding: utf-8 -*-

#from moveit_commander import robot
import rospy
import moveit_commander
from gazebo_msgs.msg import ModelStates
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32
import message_filters
import os

def setup(time, x, y, z):
    arm.set_max_velocity_scaling_factor(time)
    arm.set_max_acceleration_scaling_factor(1.0)

    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

                                                        #ハンドを動かす関数
def hand(state, time):
    gripper.set_joint_value_target([state, time])
    gripper.go()

if __name__ == "__main__":
    rospy.init_node("search")
    print("OK!!")
    main()
    rospy.spin()
