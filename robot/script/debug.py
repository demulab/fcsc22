#!/usr/bin/env python
# cooding: utf-8

import time
import sys
import math

import rospy
from std_msgs.msg import String
import smach
import smach_ros

sys.path.append('/home/demulab/opl_ws/src/fcsc22/robot/script')
from module import *
from fcsc_command import *


def main():
 #   arm = Arm()
   # ShelfCommand("low_close")
    command = XArm_command()
    command.ar_picking(61)
"""
move = Move()
    rospy.sleep(2.0)
    move.y_move(1.25)
    rospy.sleep(1.0)
    move.x_move(-1.20)
    ShelfCommand("low_open")
    rospy.sleep(10.0)
#    ShelfCommand("low_close")
#    move.x_move(1.20)
#    rospy.sleep(1.0)
#    move.y_move(-1.25)

    move.x_move(-1.25)
    rospy.sleep(2.0)
    move.x_move(1.25)
    rospy.sleep(2.0)
    move.y_move(-1.4)
"""
"""
    rate = rospy.Rate(5)
    gripper.close()
    gripper.open()
    arm.init_pose()
    
    print(vision.target_object.pose.pose)
    target_pose_1 = geometry_msgs.msg.Pose()
    target_pose_1 = vision.target_object.pose.pose
    q = tf.transformations.quaternion_from_euler(0, math.pi / 3, math.pi)
    target_pose_1.position.x = vision.target_object.pose.pose.position.x - 0.15
    target_pose_1.position.z = vision.target_object.pose.pose.position.z + 0.07
    target_pose_1.orientation.x = q[0]
    target_pose_1.orientation.y = q[1]
    target_pose_1.orientation.z = q[2]
    target_pose_1.orientation.w = q[3]
    arm.move(target_pose_1) 

    rospy.sleep(1)
    target_pose_2 = geometry_msgs.msg.Pose()
    target_pose_2 = vision.target_object.pose.pose
    q = tf.transformations.quaternion_from_euler(0, math.pi / 6, math.pi)
    target_pose_2.orientation.x = q[0]
    target_pose_2.orientation.y = q[1]
    target_pose_2.orientation.z = q[2]
    target_pose_2.orientation.w = q[3]
    arm.move(target_pose_2) 

    gripper.close()
    arm.init_pose()
    gripper.open()

    while not rospy.is_shutdown():
        print(vision.target_object.object_name)
        print(vision.target_object.pose.pose)
        arm.move(vision.target_object.pose.pose) 
        rate.sleep()

    
    move.navigation('tableB')
    move.navigation('livingB')
    move.navigation('bin')
    
    while not rospy.is_shutdown():    
        gripper.open()
        print(gripper.puressure_left, gripper.puressure_right)
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.2
        target_pose.position.y = 0
        target_pose.position.z = 0.2
        target_pose.orientation.x = 0.932
        target_pose.orientation.y = -0.06
        target_pose.orientation.z = -0.077
        target_pose.orientation.w = 0.34
        arm.move(target_pose)

        rospy.sleep(1)
        gripper.close()
        print(gripper.puressure_left, gripper.puressure_right)

        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.2
        target_pose.position.y = 0
        target_pose.position.z = 0.1
        target_pose.orientation.x = 0.93
        target_pose.orientation.y = 0
        target_pose.orientation.z = 0
        target_pose.orientation.w = 0.34
        arm.move(target_pose)
"""


if __name__ == '__main__':
    rospy.init_node("debug", anonymous=True)
    print("aaaaaaaaaaa")
    main()
"""
    try:
        main() 
    except rospy.ROSInterruptException:
        pass
"""
