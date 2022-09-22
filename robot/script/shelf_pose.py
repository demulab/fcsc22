#!/usr/bin/env python
# cooding: utf-8

import rospy
import smach
import smach_ros
import sys
import tf
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


class mikiwame(object):
    def __init__(self):
	print("Constructor")
        self.arm = Arm()
        self.gripper = Gripper()
        self.vision = Vision()
        self.arm.add_table()
        self.gripper.open()
	print("End")
    ########## set manipulator ##########    
    def shelf_mid(self):
        #self.gripper.close()
	print("Start setting")
        self.mid_trans = [0.46, 0.06, 0.53]
        self.mid_pose = geometry_msgs.msg.Pose()
        self.q = tf.transformations.quaternion_from_euler(0, (math.pi / 3)*0.3 , math.pi)
        self.mid_pose.position.x = self.mid_trans[0]
        self.mid_pose.position.y = self.mid_trans[1]
        self.mid_pose.position.z = self.mid_trans[2]
        self.mid_pose.orientation.x = self.q[0]
        self.mid_pose.orientation.y = self.q[1]
        self.mid_pose.orientation.z = self.q[2]
        self.mid_pose.orientation.w = self.q[3]
        self.arm.move(self.mid_pose)
        self.gripper.open()
	print("End setting")

    ########### grasp object ##############
    def grasp(self):
        print(self.vision.target_object.pose.pose)
        if self.vision.target_object.pose.pose.position.x == 0 and self.vision.target_object.pose.pose.position.y == 0 and self.vision.target_object.pose.pose.position.z == 0:
            return
        self.target_pose_1 = geometry_msgs.msg.Pose()
        self.target_pose_1 = self.vision.target_object.pose.pose
        self.q = tf.transformations.quaternion_from_euler(0, (math.pi / 3)*0.8, math.pi)
        
        self.target_pose_1.position.x = self.vision.target_object.pose.pose.position.x - 0.22
        self.target_pose_1.position.z = self.vision.target_object.pose.pose.position.z + 0.2#0.15
        self.target_pose_1.orientation.x = self.q[0]
        self.target_pose_1.orientation.y = self.q[1]
        self.target_pose_1.orientation.z = self.q[2]
        self.target_pose_1.orientation.w = self.q[3]
        self.arm.move(self.target_pose_1)
           
        print(self.vision.target_object.pose.pose)
        if self.vision.target_object.pose.pose.position.x == 0 and self.vision.target_object.pose.pose.position.y == 0 and self.vision.target_object.pose.pose.position.z == 0:
            return
        self.target_pose_2 = geometry_msgs.msg.Pose()
        self.target_pose_2 = self.vision.target_object.pose.pose
        self.q = tf.transformations.quaternion_from_euler(0, (math.pi / 3)*0.8, math.pi)
        self.target_pose_2.position.x = self.vision.target_object.pose.pose.position.x - 0.0#+ 0.0 #0.035 #0.03
        self.target_pose_2.position.z = self.vision.target_object.pose.pose.position.z + 0.0      #- 0.0  #0.06
        self.target_pose_2.orientation.x = self.q[0]
        self.target_pose_2.orientation.y = self.q[1]
        self.target_pose_2.orientation.z = self.q[2]
        self.target_pose_2.orientation.w = self.q[3]
        self.arm.move(self.target_pose_2)
        self.gripper.close()
    ############### bring to box ######################3
    def bring_to_box(self):
        self.box_trans = [[0.46, 0.06, 0.53], [0.3, 0.0, 0.5]]
        for way in self.box_trans:
            self.box_pose = geometry_msgs.msg.Pose()
            self.q = tf.transformations.quaternion_from_euler(0, math.pi / 3 , math.pi)
            self.box_pose.position.x = way[0]
            self.box_pose.position.y = way[1]
            self.box_pose.position.z = way[2]
            self.box_pose.orientation.x = self.q[0]
            self.box_pose.orientation.y = self.q[1]
            self.box_pose.orientation.z = self.q[2]
            self.box_pose.orientation.w = self.q[3]
            self.arm.move(self.box_pose)
        self.gripper.open()


if __name__ == "__main__":
    rospy.init_node('shelf', anonymous=True)
    robot = mikiwame()
    
    try:
	#rospy.init_node('basic_function', anonymous=True)
        robot.shelf_mid()
        rospy.sleep(15)
        robot.grasp()
        robot.bring_to_box()
    except rospy.ROSInterruptException:
        pass


