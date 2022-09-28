#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import smach
import smach_ros
import sys
import tf
import math
import numpy as np
sys.path.append('/home/demulab/opl_ws/src/fcsc22/robot/script')
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
########## AR msg ##########
from ar_track_alvar_msgs.msg import AlvarMarkers




class XArm_command(object):
    def __init__(self):
	print("Constructor")
        self.arm = Arm()
        self.gripper = Gripper()
        self.vision = Vision()
        self.arm.add_table()
        #self.gripper.open()
        self.gripper.vacuum_off()
        self.listener = tf.TransformListener()
	print("End")
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.ar_cb)

        self.ar_info = []
########## set manipulator ##########        
    def shelf_mid(self):
        #self.gripper.close()
	print("Start setting")
        self.mid_trans = [0.39, 0.06, 0.53]
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



    def setting_pose(self, trans):
        print("start setting")
        tr = trans
        self.pose = geometry_msgs.msg.Pose()
         #self.q = tf.transformations.quaternion_from_euler(0, (math.pi / 3)*0.3 , math.pi)

        self.q = tf.transformations.quaternion_from_euler(0, (math.pi/ 3)*0.4, math.pi)
        self.pose.position.x = tr[0]
        self.pose.position.y = tr[1]
        self.pose.position.z = tr[2]
        self.pose.orientation.x = self.q[0]
        self.pose.orientation.y = self.q[1]
        self.pose.orientation.z = self.q[2]
        self.pose.orientation.w = self.q[3]
        print(self.pose)
        self.arm.move(self.pose)



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
        add_height = 0.0

        self.box_trans =  [[0.30, 0.0, 0.45 + add_height], [0.15, 0.20, 0.45 + add_height], [0.0, 0.35, 0.45 + add_height], [-0.24, 0.0, 0.45]]
        self.num = len(self.box_trans)
        #self.q = tf.transformations.quaternion_from_euler(0, math.pi / 3 , math.pi/self.num)
        
        for index, way in enumerate(self.box_trans):
            self.box_pose = geometry_msgs.msg.Pose()
            self.q = tf.transformations.quaternion_from_euler(0, 0 , math.pi)
            self.box_pose.position.x = way[0]
            self.box_pose.position.y = way[1]
            self.box_pose.position.z = way[2]
            self.box_pose.orientation.x = self.q[0]
            self.box_pose.orientation.y = self.q[1]
            self.box_pose.orientation.z = self.q[2]
            self.box_pose.orientation.w = self.q[3]
            self.arm.move(self.box_pose)
        self.gripper.vacuum_off()


    
    def ar_picking(self, id_num):
        """
        @brief　(旧)ARのピッキング

        """        
        debug = False
        tf_diff = 0.0
        if debug == True:
            tf_diff = -0.1

        ID = id_num
        self.ar_name = "ar_marker_" + str(ID)
        self.listener.waitForTransform("/camera_link", self.ar_name, rospy.Time(), rospy.Duration(10.0))
        (trans,rot) = self.listener.lookupTransform('/world', '/%s' % (self.ar_name), rospy.Time(0))
        self.target_pose = geometry_msgs.msg.Pose()
        
        #self.target_pose = self.vision.target_object.pose.pose
        self.q = tf.transformations.quaternion_from_euler(0, (math.pi / 3)*0.8, math.pi)
 
        self.target_pose.position.x = trans[0] - 0 + tf_diff
        self.target_pose.position.y = trans[1]
        self.target_pose.position.z = trans[2] + 0.1#0.15
        self.target_pose.orientation.x = self.q[0]#rot[0]
        self.target_pose.orientation.y = self.q[1]#rot[1]
        self.target_pose.orientation.z = self.q[2]#rot[2]
        self.target_pose.orientation.w = self.q[3]#rot[3]
        print(self.target_pose)

        self.arm.move(self.target_pose)
        self.gripper.close()



    def look_shelf(self, step, side):
        print("############################################")
        if step == "low":
            if side =="left":
                self.setting_pose([0.30, 0.17, 0.48])
            if side == "right":
                self.setting_pose([0.30, -0.17, 0.48])
        elif step == "middle":
            if side =="left":
                 self.setting_pose([0.30, 0.20, 0.59])
            if side == "right":
                 self.setting_pose([0.30, -0.20, 0.59])
        #elif step == "high":


    def calc_z_axis_direction(self, roll, pitch, yaw):
    
        init_z = np.array([[0, 0, 1]]).T
    
        rot_x = np.array([[1,      0,             0      ],
                          [0, np.cos(roll), -np.sin(roll)],
                          [0, np.sin(roll),   np.cos(roll)]])
                      
        rot_y = np.array([[ np.cos(pitch), 0, np.sin(pitch)],
                          [      0,        1,      0       ],
                          [-np.sin(pitch), 0, np.cos(pitch)]])
                      
        rot_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                          [np.sin(yaw),  np.cos(yaw), 0],
                          [     0,            0,      1]])
                      
        rot_mat = np.dot(rot_z, np.dot(rot_y, rot_x))
        trans_z = np.dot(rot_mat, init_z)
        return trans_z


    def ar_cb(self, msg):
        """
        @brief　確認用のコード兼インスタンス変数へのデータ格納

        """
        """ 
        for i in msg.markers:
            print(i.id)
            print(i.pose)
            print(i.pose.pose.orientation.x)
       """ 

        self.ar_info = msg.markers


    def vacuum_picking(self, ar_id):
        """
        @brief　バキュームによるARのピッキング（垂直侵入）
 
        """
        print("ar_vacuum")
        subject = ar_id
        self.ar_name = "ar_marker_" + str(subject)

        """
        （（（（やりたい全体の流れ））））

        ARマーカのリストの目当てのidを持つデータを抽出
        　　　　　　　　　　　↓
        つかみやすい姿勢を持つマーカを選定(?)
        　　　　　　　　　　　↓
                            把持
        """
        # tfの待機
        self.listener.waitForTransform("/camera_link", self.ar_name, rospy.Time(), rospy.Duration(10.0))

       #idの一致するデータの探索 
        for d in self.ar_info:
            if d.id == subject:
                sub_trans  = d.pose.pose 
       
        ox = sub_trans.orientation.x
        oy = sub_trans.orientation.y
        oz = sub_trans.orientation.z
        ow = sub_trans.orientation.w
        
        eular = tf.transformations.euler_from_quaternion((ox, oy, oz, ow))
        """
        vec_z = self.calc_z_axis_direction(eular[0], eular[1], eular[2])
        #print(eular)
        #print(vec_z)
        
        offset = 0.03
        
        up_point = offset * vec_z
        print(up_point)
        """
        # tfを待つスリープ
        rospy.sleep(10)
        self.target_pose = geometry_msgs.msg.Pose()
        (trans,rot) = self.listener.lookupTransform('/world', '/%s' % (self.ar_name), rospy.Time(0))
        # 垂直方向の計算
        euler = tf.transformations.euler_from_quaternion(rot)
        vec_z = self.calc_z_axis_direction(euler[0], euler[1], euler[2])
        offset = 0.03
        up_point = offset * vec_z
        self.q = rot

        # approach 1
        self.target_pose.position.x = trans[0] + (up_point[0][0] * 2.5)
        self.target_pose.position.y = trans[1] + (up_point[1][0] * 2.5)
        self.target_pose.position.z = trans[2] + (up_point[2][0] * 2.5)
        self.target_pose.orientation.x = self.q[0]#rot[0]
        self.target_pose.orientation.y = self.q[1]#rot[1]
        self.target_pose.orientation.z = self.q[2]#rot[2]
        self.target_pose.orientation.w = self.q[3]#rot[3]
        print(self.target_pose)
        self.arm.move(self.target_pose)

        # approach 2
        self.target_pose.position.x = trans[0] #+ up_point[0][0]
        self.target_pose.position.y = trans[1] #+ up_point[1][0]
        self.target_pose.position.z = trans[2] #+ up_point[2][0]
        self.target_pose.orientation.x = self.q[0]#rot[0]
        self.target_pose.orientation.y = self.q[1]#rot[1]
        self.target_pose.orientation.z = self.q[2]#rot[2]
        self.target_pose.orientation.w = self.q[3]#rot[3]
        print(self.target_pose)

        self.arm.move(self.target_pose)
        self.gripper.vacuum_on()
        


if __name__ == "__main__":
    rospy.init_node('shelf', anonymous=True)
    robot = XArm_command()
    
    try:
        
        #robot.look_shelf("low", "right")
        #robot.ar_picking()
        robot.look_shelf("low", "right")
        #rospy.sleep(15)
        #robot.grasp()
        #robot.bring_to_box()
        robot.vacuum_picking(0)
        robot.bring_to_box()
        robot .look_shelf("low", "right")
        print("vacuum end")
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass


