#!/usr/bin/env python
# cooding: utf-8

import rospy
import smach
import smach_ros
import sys

sys.path.append('/home/demulab/opl_ws/src/OPL22/robot/script')
from module import *
import rosparam
import os
from std_msgs.msg import String, Float64, Bool
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
sys.path.append('/home/demulab/scout_ws/src/scout_ros/scout_navigation/srv')
from scout_navigation.srv import NaviLocation, NaviLocationResponse
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def main(): 
    arm.add_table()
    arm.init_pose()
    gripper.open()
    
    print(vision.target_object.pose.pose)
    if vision.target_object.pose.pose.position.x == 0 and vision.target_object.pose.pose.position.y == 0 and vision.target_object.pose.pose.position.z == 0:
        return
    target_pose_1 = geometry_msgs.msg.Pose()
    target_pose_1 = vision.target_object.pose.pose
    q = tf.transformations.quaternion_from_euler(0, math.pi / 3, math.pi)
    target_pose_1.position.x = vision.target_object.pose.pose.position.x - 0.2
    target_pose_1.position.z = vision.target_object.pose.pose.position.z + 0.0
    target_pose_1.orientation.x = q[0]
    target_pose_1.orientation.y = q[1]
    target_pose_1.orientation.z = q[2]
    target_pose_1.orientation.w = q[3]
    arm.move(target_pose_1)             

    print(vision.target_object.pose.pose)
    if vision.target_object.pose.pose.position.x == 0 and vision.target_object.pose.pose.position.y == 0 and vision.target_object.pose.pose.position.z == 0:
        return
    target_pose_2 = geometry_msgs.msg.Pose()
    target_pose_2 = vision.target_object.pose.pose
    q = tf.transformations.quaternion_from_euler(0, math.pi / 3, math.pi)
    target_pose_2.position.x = vision.target_object.pose.pose.position.x + 0.0 #0.035 #0.03
    target_pose_2.position.z = vision.target_object.pose.pose.position.z - 0.01  #0.06
    target_pose_2.orientation.x = q[0]
    target_pose_2.orientation.y = q[1]
    target_pose_2.orientation.z = q[2]
    target_pose_2.orientation.w = q[3]
    arm.move(target_pose_2) 
    gripper.close()

    arm.init_pose()
    gripper.open()

if __name__ == '__main__':
    arm = Arm()
    gripper = Gripper()
    vision = Vision()
    
    try:
        rospy.init_node('open_campus', anonymous=True)
        main()
    except rospy.ROSInterruptException:
        pass
