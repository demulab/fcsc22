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



class PickAndPlace():
    def __init__(self):
        pass

    def pick(self):
        print(vision.target_object.pose.pose)
        if vision.target_object.pose.pose.position.x == 0 and vision.target_object.pose.pose.position.y == 0 and vision.target_object.pose.pose.position.z == 0:
            return
        target_pose_1 = geometry_msgs.msg.Pose()
        target_pose_1 = vision.target_object.pose.pose
        q = tf.transformations.quaternion_from_euler(0, math.pi / 4, math.pi)
        target_pose_1.position.x = vision.target_object.pose.pose.position.x - 0.2
        target_pose_1.position.y = vision.target_object.pose.pose.position.y - 0.03
        target_pose_1.position.z = vision.target_object.pose.pose.position.z + 0.02
        target_pose_1.orientation.x = q[0]
        target_pose_1.orientation.y = q[1]
        target_pose_1.orientation.z = q[2]
        target_pose_1.orientation.w = q[3]
        arm.move(target_pose_1) 

        rospy.sleep(3)
        target_pose_2 = geometry_msgs.msg.Pose()
        if vision.target_object.pose.pose.position.x == 0 and vision.target_object.pose.pose.position.y == 0 and vision.target_object.pose.pose.position.z == 0:
            return
        target_pose_2 = vision.target_object.pose.pose
        q = tf.transformations.quaternion_from_euler(0, math.pi / 3, math.pi)
        target_pose_2.position.x = vision.target_object.pose.pose.position.x
        target_pose_2.position.y = vision.target_object.pose.pose.position.y
        target_pose_2.position.z = vision.target_object.pose.pose.position.z - 0.04
        target_pose_2.orientation.x = q[0]
        target_pose_2.orientation.y = q[1]
        target_pose_2.orientation.z = q[2]
        target_pose_2.orientation.w = q[3]
        arm.move(target_pose_2) 

    def place(self):
        target_pose_3 = geometry_msgs.msg.Pose()
        q = tf.transformations.quaternion_from_euler(0, math.pi / 2, math.pi)
        target_pose_3.position.x = 0.8
        target_pose_3.position.y = 0.0
        target_pose_3.position.z = 0.27
        target_pose_3.orientation.x = q[0]
        target_pose_3.orientation.y = q[1]
        target_pose_3.orientation.z = q[2]
        target_pose_3.orientation.w = q[3]
        arm.move(target_pose_3)

    def place_for_left(self):
        target_pose_3 = geometry_msgs.msg.Pose()
        q = tf.transformations.quaternion_from_euler(-math.pi / 4, math.pi / 3, math.pi)
        target_pose_3.position.x = 0.5
        target_pose_3.position.y = 0.4
        target_pose_3.position.z = 0.3
        target_pose_3.orientation.x = q[0]
        target_pose_3.orientation.y = q[1]
        target_pose_3.orientation.z = q[2]
        target_pose_3.orientation.w = q[3]
        arm.move(target_pose_3)

    def place_for_right(self):
        target_pose_3 = geometry_msgs.msg.Pose()
        q = tf.transformations.quaternion_from_euler(math.pi / 4, math.pi / 3, math.pi)
        target_pose_3.position.x = 0.5
        target_pose_3.position.y = -0.4
        target_pose_3.position.z = 0.3
        target_pose_3.orientation.x = q[0]
        target_pose_3.orientation.y = q[1]
        target_pose_3.orientation.z = q[2]
        target_pose_3.orientation.w = q[3]
        arm.move(target_pose_3)

        
def main():
    voice.play("start_tidyup.mp3")
    arm.init_pose()
    voice.play("please_open_the_door.mp3")
    rospy.sleep(6.0)
    move.x_move(1.35)
    voice.play("start_task2A.mp3")
    move.navigation('dining')
    move.x_move(0.8)
    voice.play("start_task2B.mp3")
    move.navigation('shelf')
    rospy.sleep(5.0)
    
    arm.init_pose()
    gripper.open()
    gripper.close()
    gripper.open()
    arm.add_table()
    pick.pick()
    gripper.close()
    rospy.sleep(1.0)
    move.y_move(0.25)
    rospy.sleep(1)
    arm.init_pose()

    rospy.sleep(1.0)
    move.x_move(-1.6)
    # voice.play("Those_on_the_right_please_extend_your_hand.mp3")
    # pick.place_for_right()
    voice.play("Those_on_the_left_please_extend_your_hand.mp3")
    pick.place_for_left()
    rospy.sleep(1)
    gripper.open() 
    arm.init_pose()

if __name__ == '__main__':
    arm = Arm()
    gripper = Gripper()
    vision = Vision()
    voice = Voice()
    move = Move()
    pick = PickAndPlace()

    try:
        rospy.init_node('basic_function', anonymous=True)
        main()
    except rospy.ROSInterruptException:
        pass
