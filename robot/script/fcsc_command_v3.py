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
        #self.arm.add_table()
        #self.gripper.open()
        self.gripper.vacuum_off()
        self.listener = tf.TransformListener()
	print("End")
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.ar_cb)
        self.ar_info = []
        self.all_id = [range(1, 7), range(7, 13), range(13, 19)]
        


    def setting_pose(self, trans, heights):
        print("start setting")
        tr = trans
        self.pose = geometry_msgs.msg.Pose()
         #self.q = tf.transformations.quaternion_from_euler(0, (math.pi / 3)*0.3 , math.pi)
        if heights == "high":
            self.q = tf.transformations.quaternion_from_euler(0, (math.pi/ 3)*0.6, math.pi)
        elif heights == "low":
            self.q = tf.transformations.quaternion_from_euler(0, (math.pi/ 3)*0.5, math.pi)
        else:
            self.q = tf.transformations.quaternion_from_euler(0, (math.pi/ 3)*0.6, math.pi)
        self.pose.position.x = tr[0]
        self.pose.position.y = tr[1]
        self.pose.position.z = tr[2]
        self.pose.orientation.x = self.q[0]
        self.pose.orientation.y = self.q[1]
        self.pose.orientation.z = self.q[2]
        self.pose.orientation.w = self.q[3]
        print(self.pose)
        self.arm.move(self.pose)


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


    def bring_to_box(self):
        """
        @brief 把持後のボックスへの運搬

        """
        add_height = 0.0

        #self.box_trans =  [[0.30, 0.0, 0.68 + add_height], [0.15, 0.20, 0.45 + add_height], [0.0, 0.35, 0.45 + add_height], [-0.24, 0.0, 0.45]]
        self.box_trans =  [[0.1, 0.10, 0.55 + add_height], [-0.3, 0.1, 0.3]]
        self.num = len(self.box_trans)
        
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

    def coffee_bring_to_box(self):
        """
        @brief 把持後のボックスへの運搬

        """
        add_height = 0.0

        #self.box_trans =  [[0.30, 0.0, 0.68 + add_height], [0.15, 0.20, 0.45 + add_height], [0.0, 0.35, 0.45 + add_height], [-0.24, 0.0, 0.45]]
        self.box_trans =  [[0.1, 0.10, 0.55 + add_height], [-0.3, 0.1, 0.3]]
        self.num = len(self.box_trans)
        
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



    def look_shelf(self, step, side):
        if step == "low":
            if side =="left":
                #self.setting_pose([0.30, 0, 0.48])
                self.setting_pose([0.14, 0.16, -0.14], "low")
            if side == "right":
                #self.setting_pose([0.30, 0, 0.48])
                self.setting_pose([0.14, -0.30, -0.14], "low")
        elif step == "middle":
            if side =="left":
                #self.setting_pose([0.30, 0, 0.59])
                self.setting_pose([0.10, 0.12, 0.25], "middle")
            if side == "right":
                #self.setting_pose([0.30, 0, 0.59])
                self.setting_pose([0.10, -0.3, 0.25], "middle")
        elif step == "high":
            if side == "left":
                #self.setting_pose([0.30, 0, 0.70])
                self.setting_pose([0.1, 0.11, 0.53], "high")
            if side == "right":
                #self.setting_pose([0.30, 0, 0.70])
                self.setting_pose([0.1, -0.2, 0.53],"high")
        

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


    def calc_x_axis_direction(self, roll, pitch, yaw):

        init_x = np.array([[1, 0, 0]]).T
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
        trans_x = np.dot(rot_mat, init_x)
        return trans_x

    def calc_y_axis_direction(self, roll, pitch, yaw):

        init_y = np.array([[0, 1, 0]]).T
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
        trans_y = np.dot(rot_mat, init_y)
        return trans_y



    def ar_cb(self, msg):
        """
        @brief　確認用のコード兼インスタンス変数へのデータ格納

        """
        #print("callback")
        self.ar_info = msg.markers



    def get_optimul_ar(self, ids):
        """
        @brief 吸着に最適なマーカの選定
        
        """
        prev_x = 180
        prev_y = 180
        opt_x = 180
        opt_y = 180
        opt_id = 0
        for data in ids:
            print("######################################################")
            print("processing", str(data))
            #print(opt_x, opt_y)
            try:
                ar = "ar_marker_" + str(data)
                print(ar)
                self.listener.waitForTransform("/camera_link", ar, rospy.Time(), rospy.Duration(2.0))
                (trans,rot) = self.listener.lookupTransform('/world', '/%s' % (ar), rospy.Time(0))
                print("transformed")
                euler = tf.transformations.euler_from_quaternion(rot)
                ex = abs(euler[0] * (180 / math.pi))
                ey = abs(euler[1] * (180 / math.pi))
                print(ex, ey)
                if ex < 20 and ey < 20:
                    if ex + ey < prev_x + prev_y:
                        if True:
                            print("updated",ex, ey)
                            opt_id = int(data)
                            prev_x = ex
                            prev_y = ey
                            opt_x = ex
                            opt_y = ey
                            
                            print("id_", str(data), "_success!!!")
                            continue
                            #break
                else:
                    pass
            except:
                pass
        if opt_id == 0:
            print("passed")
            f = False
            return f

        else:
            print(opt_id)
            print("vacuuuuuuuuuuuuuuuuuum")
            self.vacuum_picking(opt_id)
            f = True
            return f
            #pass



    def sand_get_optimul_ar(self, ids):
        """
        @brief 吸着に最適なマーカの選定
        
        """
        prev_x = 180
        prev_y = 180
        opt_x = 180
        opt_y = 180
        opt_id = 0
        for data in ids:
            print("######################################################")
            print("processing", str(data))
            #print(opt_x, opt_y)
            try:
                ar = "ar_marker_" + str(data)
                print(ar)
                self.listener.waitForTransform("/camera_link", ar, rospy.Time(), rospy.Duration(2.0))
                (trans,rot) = self.listener.lookupTransform('/world', '/%s' % (ar), rospy.Time(0))
                print("transformed")
                euler = tf.transformations.euler_from_quaternion(rot)
                ex = abs(euler[0] * (180 / math.pi))
                ey = abs(euler[1] * (180 / math.pi))
                print(ex, ey)
                if ex < 60 and ey < 60:
                    if ex + ey < prev_x + prev_y:
                        if True:
                            print("updated",ex, ey)
                            opt_id = int(data)
                            prev_x = ex
                            prev_y = ey
                            opt_x = ex
                            opt_y = ey
                            
                            print("id_", str(data), "_success!!!")
                            continue
                            #break
                else:
                    pass
            except:
                pass
        if opt_id == 0:
            print("passed")
            f = False
            return f

        else:
            print(opt_id)
            print("vacuuuuuuuuuuuuuuuuuum")
            self.sand_vacuum_picking(opt_id)
            f = True
            return f
            #pass



    def coffee_get_optimul_ar(self, ids):
        """
        @brief 吸着に最適なマーカの選定
        
        """
        prev_x = 180
        prev_y = 180
        opt_x = 180
        opt_y = 180
        opt_id = 0
        for data in ids:
            print("######################################################")
            print("processing", str(data))
            #print(opt_x, opt_y)
            try:
                ar = "ar_marker_" + str(data)
                print(ar)
                self.listener.waitForTransform("/camera_link", ar, rospy.Time(), rospy.Duration(2.0))
                (trans,rot) = self.listener.lookupTransform('/world', '/%s' % (ar), rospy.Time(0))
                print("transformed")
                euler = tf.transformations.euler_from_quaternion(rot)
                ex = abs(euler[0] * (180 / math.pi))
                ey = abs(euler[1] * (180 / math.pi))
                print(ex, ey)
                if ex < 48 and ey < 48:
                    if ex + ey < prev_x + prev_y:
                        if True:
                            print("updated",ex, ey)
                            opt_id = int(data)
                            prev_x = ex
                            prev_y = ey
                            opt_x = ex
                            opt_y = ey
                            
                            print("id_", str(data), "_success!!!")
                            continue
                            #break
                else:
                    pass
            except:
                pass
        if opt_id == 0:
            print("passed")
            f = False
            return f

        else:
            print(opt_id)
            print("vacuuuuuuuuuuuuuuuuuum")
            self.coffee_vacuum_picking(opt_id)
            f = True
            return f
            





    def exist_ar(self, ids):
        exist = 0
        rospy.sleep(3)
        # ar_track topic
        print(self.ar_info)
        for data in self.ar_info:
            #print(data)
            #print(type(data.id))
            if data.id in ids:
                # debug
                # print("##########\nTrue\n##########")
                print(data.id)
                exist = 1
                break
            else:
                continue

        # tf topic
        

        if exist == 1:
            print("##########\n\n\ndetected\n\n\n##########")
            f = True
            return f
        else:
            print("##########\n\n\nNone\n\n\n##########")
            f = False
            return f

    def vacuum_picking(self, ar_id):
        """
        @brief　バキュームによるARのピッキング
 
        """
        print("ar_vacuum")
        self.ar_name = "ar_marker_" + str(ar_id)
        # tfの待機
        self.listener.waitForTransform("/camera_link", self.ar_name, rospy.Time(), rospy.Duration(10.0))
        self.target_pose = geometry_msgs.msg.Pose()
        (trans,rot) = self.listener.lookupTransform('/world', '/%s' % (self.ar_name), rospy.Time(0))
        # 垂直方向の計算
        euler = tf.transformations.euler_from_quaternion(rot)
        vec_z = self.calc_z_axis_direction(euler[0], euler[1], euler[2])
        offset = 0.03
        up_point = offset * vec_z
        self.q = tf.transformations.quaternion_from_euler(0, math.pi/8 , math.pi)
        # approach 1 マーカに簡単な位置合わせ
        self.target_pose.position.x = trans[0] - 0.4 #(10 * up_point[0][0]) - 0.3
        self.target_pose.position.y = trans[1] + 0.0#(10 * up_point[1][0]) #- 0.02
        self.target_pose.position.z = trans[2] + 0.3#(10 * up_point[2][0])
        self.target_pose.orientation.x = self.q[0]#rot[0]
        self.target_pose.orientation.y = self.q[1]#rot[1]
        self.target_pose.orientation.z = self.q[2]#rot[2]
        self.target_pose.orientation.w = self.q[3]#rot[3]
        print(self.target_pose)
        #self.arm.move(self.target_pose)
        
        (trans,rot) = self.listener.lookupTransform('/world', '/%s' % (self.ar_name), rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
        vec_z = self.calc_z_axis_direction(euler[0], euler[1], euler[2])
        vec_x = self.calc_x_axis_direction(euler[0], euler[1], euler[2])
        vec_y = self.calc_y_axis_direction(euler[0], euler[1], euler[2])
        
        offset = 0.03
        x_offset = 0.03
        y_offset = 0.03
        up_point = offset * vec_z
        x_point = x_offset * vec_x
        y_point = y_offset * vec_y

        #rot  = tf.transformations.quaternion_from_euler(euler[0], euler[1] , math.pi)
        
        self.q = rot
        # approach 2
        self.target_pose.position.x = trans[0] - (0.35 * up_point[0][0]) - 0.014#- 0.02 #+ (0 * x_point[0][0]) - (0.4 * y_point[0][0])
        self.target_pose.position.y = trans[1] - (0.35 * up_point[1][0]) - 0.032#+ (0 * x_point[1][0]) - (0.4 * y_point[1][0]) #- 0.03#(1.89 * up_point[1][0]) - 0.04
        self.target_pose.position.z = trans[2] - (0.35 * up_point[2][0]) #+ (0 * x_point[2][0]) - (0.4 * y_point[2][0])
        self.target_pose.orientation.x = self.q[0]#rot[0]
        self.target_pose.orientation.y = self.q[1]#rot[1]
        self.target_pose.orientation.z = self.q[2]#rot[2]
        self.target_pose.orientation.w = self.q[3]#rot[3]
        print(self.target_pose)
        self.arm.move(self.target_pose)

        #final approach
        self.target_pose.position.x = trans[0] - (0.85 * up_point[0][0]) - 0.014#- 0.02#+ (0 * x_point[0][0]) - (0.4 * y_point[0][0]) #- 0.01#(1.89 * up_point[0][0]) - 0.02
        self.target_pose.position.y = trans[1] - (0.85 * up_point[1][0]) - 0.032#+ (0 * x_point[1][0]) - (0.4 * y_point[1][0]) #- 0.03#(1.89 * up_point[1][0]) - 0.04
        self.target_pose.position.z = trans[2] - (0.85 * up_point[2][0]) #+ (0 * x_point[2][0]) - (0.4 * y_point[2][0])
        self.target_pose.orientation.x = self.q[0]#rot[0]
        self.target_pose.orientation.y = self.q[1]#rot[1]
        self.target_pose.orientation.z = self.q[2]#rot[2]
        self.target_pose.orientation.w = self.q[3]#rot[3]
        print(self.target_pose)
        self.arm.move(self.target_pose)
        #self.gripper.vacuum_on()
        

    def coffee_vacuum_picking(self, ar_id):
        """
        @brief　バキュームによるARのピッキング
 
        """
        print("ar_vacuum")
        self.ar_name = "ar_marker_" + str(ar_id)
        # tfの待機
        self.listener.waitForTransform("/camera_link", self.ar_name, rospy.Time(), rospy.Duration(10.0))
        self.target_pose = geometry_msgs.msg.Pose()
        (trans,rot) = self.listener.lookupTransform('/world', '/%s' % (self.ar_name), rospy.Time(0))
        # 垂直方向の計算
        euler = tf.transformations.euler_from_quaternion(rot)
        vec_z = self.calc_z_axis_direction(euler[0], euler[1], euler[2])
        offset = 0.03
        up_point = offset * vec_z
        self.q = tf.transformations.quaternion_from_euler(0, math.pi/8 , math.pi)
        # approach 1 マーカに簡単な位置合わせ
        self.target_pose.position.x = trans[0] - 0.4 #(10 * up_point[0][0]) - 0.3
        self.target_pose.position.y = trans[1] + 0.0#(10 * up_point[1][0]) #- 0.02
        self.target_pose.position.z = trans[2] + 0.3#(10 * up_point[2][0])
        self.target_pose.orientation.x = self.q[0]#rot[0]
        self.target_pose.orientation.y = self.q[1]#rot[1]
        self.target_pose.orientation.z = self.q[2]#rot[2]
        self.target_pose.orientation.w = self.q[3]#rot[3]
        print(self.target_pose)
        #self.arm.move(self.target_pose)
        
        (trans,rot) = self.listener.lookupTransform('/world', '/%s' % (self.ar_name), rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
        vec_z = self.calc_z_axis_direction(euler[0], euler[1], euler[2])
        vec_x = self.calc_x_axis_direction(euler[0], euler[1], euler[2])
        vec_y = self.calc_y_axis_direction(euler[0], euler[1], euler[2])
        
        offset = 0.03
        x_offset = 0.03
        y_offset = 0.03
        up_point = offset * vec_z
        x_point = x_offset * vec_x
        y_point = y_offset * vec_y

        #rot  = tf.transformations.quaternion_from_euler(euler[0], euler[1] , math.pi)
        
        self.q = rot
        # approach 2
        self.target_pose.position.x = trans[0] - (0.35 * up_point[0][0]) - 0.014#- 0.02 #+ (0 * x_point[0][0]) - (0.4 * y_point[0][0])
        self.target_pose.position.y = trans[1] - (0.35 * up_point[1][0]) - 0.033#+ (0 * x_point[1][0]) - (0.4 * y_point[1][0]) #- 0.03#(1.89 * up_point[1][0]) - 0.04
        self.target_pose.position.z = trans[2] - (0.35 * up_point[2][0]) #+ (0 * x_point[2][0]) - (0.4 * y_point[2][0])
        self.target_pose.orientation.x = self.q[0]#rot[0]
        self.target_pose.orientation.y = self.q[1]#rot[1]
        self.target_pose.orientation.z = self.q[2]#rot[2]
        self.target_pose.orientation.w = self.q[3]#rot[3]
        print(self.target_pose)
        self.arm.move(self.target_pose)

        # final approach
        self.target_pose.position.x = trans[0] - (0.85 * up_point[0][0]) - 0.014#- 0.02#+ (0 * x_point[0][0]) - (0.4 * y_point[0][0]) #- 0.01#(1.89 * up_point[0][0]) - 0.02
        self.target_pose.position.y = trans[1] - (0.85 * up_point[1][0]) - 0.033#+ (0 * x_point[1][0]) - (0.4 * y_point[1][0]) #- 0.03#(1.89 * up_point[1][0]) - 0.04
        self.target_pose.position.z = trans[2] - (0.85 * up_point[2][0]) #+ (0 * x_point[2][0]) - (0.4 * y_point[2][0])
        self.target_pose.orientation.x = self.q[0]#rot[0]
        self.target_pose.orientation.y = self.q[1]#rot[1]
        self.target_pose.orientation.z = self.q[2]#rot[2]
        self.target_pose.orientation.w = self.q[3]#rot[3]
        print(self.target_pose)
        self.arm.move(self.target_pose)
        #self.gripper.vacuum_on()
        """
        # pull up
        self.target_pose.position.x = trans[0] + (2 * up_point[0][0]) - 0.005#- 0.02#+ (0 * x_point[0][0]) - (0.4 * y_point[0][0]) #- 0.01#(1.89 * up_point[0][0]) - 0.02
        self.target_pose.position.y = trans[1] + (2 * up_point[1][0]) - 0.033#+ (0 * x_point[1][0]) - (0.4 * y_point[1][0]) #- 0.03#(1.89 * up_point[1][0]) - 0.04
        self.target_pose.position.z = trans[2] + (2 * up_point[2][0]) #+ (0 * x_point[2][0]) - (0.4 * y_point[2][0])
        self.target_pose.orientation.x = self.q[0]#rot[0]
        self.target_pose.orientation.y = self.q[1]#rot[1]
        self.target_pose.orientation.z = self.q[2]#rot[2]
        self.target_pose.orientation.w = self.q[3]#rot[3]
        print(self.target_pose)
        self.arm.move(self.target_pose)
        """


    def coffee_pullup(self):
        # pull up
        #self.target_pose.position.x = trans[0] + (3 * up_point[0][0]) - 0.005#- 0.02#+ (0 * x_point[    0][0]) - (0.4 * y_point[0][0]) #- 0.01#(1.89 * up_point[0][0]) - 0.02
        #self.target_pose.position.y = trans[1] + (3 * up_point[1][0]) - 0.033#+ (0 * x_point[1][0])     - (0.4 * y_point[1][0]) #- 0.03#(1.89 * up_point[1][0]) - 0.04
        self.target_pose.position.z += 0.2#+ (0 * x_point[2][0]) - (0.4     * y_point[2][0])
        print(self.target_pose)
        self.arm.move(self.target_pose)


    def sand_vacuum_picking(self, ar_id):
        """
        @brief　バキュームによるARのピッキング
 
        """
        print("ar_vacuum")
        self.ar_name = "ar_marker_" + str(ar_id)
        # tfの待機
        self.listener.waitForTransform("/camera_link", self.ar_name, rospy.Time(), rospy.Duration(10.0))
        self.target_pose = geometry_msgs.msg.Pose()
        (trans,rot) = self.listener.lookupTransform('/world', '/%s' % (self.ar_name), rospy.Time(0))
        # 垂直方向の計算
        euler = tf.transformations.euler_from_quaternion(rot)
        vec_z = self.calc_z_axis_direction(euler[0], euler[1], euler[2])
        offset = 0.03
        up_point = offset * vec_z
        self.q = tf.transformations.quaternion_from_euler(0, math.pi/8 , math.pi)
        (trans,rot) = self.listener.lookupTransform('/world', '/%s' % (self.ar_name), rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
        vec_z = self.calc_z_axis_direction(euler[0], euler[1], euler[2])
        vec_x = self.calc_x_axis_direction(euler[0], euler[1], euler[2])
        vec_y = self.calc_y_axis_direction(euler[0], euler[1], euler[2])
        
        offset = 0.03
        x_offset = 0.03
        y_offset = 0.03
        up_point = offset * vec_z
        x_point = x_offset * vec_x
        y_point = y_offset * vec_y

        #rot  = tf.transformations.quaternion_from_euler(euler[0], euler[1] , math.pi)
        
        self.q = rot
        # approach 2
        self.target_pose.position.x = trans[0] + (1.6 * up_point[0][0]) - 0.006#- 0.02 #+ (0 * x_point[0][0]) - (0.4 * y_point[0][0])
        self.target_pose.position.y = trans[1] + (1.6 * up_point[1][0]) - 0.033#+ (0 * x_point[1][0]) - (0.4 * y_point[1][0]) #- 0.03#(1.89 * up_point[1][0]) - 0.04
        self.target_pose.position.z = trans[2] + (1.6 * up_point[2][0]) #+ (0 * x_point[2][0]) - (0.4 * y_point[2][0])
        self.target_pose.orientation.x = self.q[0]#rot[0]
        self.target_pose.orientation.y = self.q[1]#rot[1]
        self.target_pose.orientation.z = self.q[2]#rot[2]
        self.target_pose.orientation.w = self.q[3]#rot[3]
        print(self.target_pose)
        self.arm.move(self.target_pose)

        #final approach
        self.target_pose.position.x = trans[0] + (0 * up_point[0][0]) - 0.006#- 0.02#+ (0 * x_point[0][0]) - (0.4 * y_point[0][0]) #- 0.01#(1.89 * up_point[0][0]) - 0.02
        self.target_pose.position.y = trans[1] + (0 * up_point[1][0]) - 0.033#+ (0 * x_point[1][0]) - (0.4 * y_point[1][0]) #- 0.03#(1.89 * up_point[1][0]) - 0.04
        self.target_pose.position.z = trans[2] + (0 * up_point[2][0]) #+ (0 * x_point[2][0]) - (0.4 * y_point[2][0])
        self.target_pose.orientation.x = self.q[0]#rot[0]
        self.target_pose.orientation.y = self.q[1]#rot[1]
        self.target_pose.orientation.z = self.q[2]#rot[2]
        self.target_pose.orientation.w = self.q[3]#rot[3]
        print(self.target_pose)
        self.arm.move(self.target_pose)
        #self.gripper.vacuum_on()
    



    def planning_vertical(self, trans, right_angle):
        print("start setting")
        tr = trans
        self.pose = geometry_msgs.msg.Pose()
        #直角か否か
        if right_angle == True:
            self.q = tf.transformations.quaternion_from_euler(0, 0, math.pi/2)
        else:
            self.q = tf.transformations.quaternion_from_euler(0, 0, math.pi)
        self.pose.position.x = tr[0]-0.03
        self.pose.position.y = tr[1]-0.04
        self.pose.position.z = tr[2]+0.06 # debug
        self.pose.orientation.x = self.q[0]
        self.pose.orientation.y = self.q[1]
        self.pose.orientation.z = self.q[2]
        self.pose.orientation.w = self.q[3]
        print(self.pose)
        self.arm.move(self.pose)
        
        self.pose.position.x = tr[0]-0.03
        self.pose.position.y = tr[1]-0.04
        self.pose.position.z = tr[2]-0.02 # debug
        self.pose.orientation.x = self.q[0]
        self.pose.orientation.y = self.q[1]
        self.pose.orientation.z = self.q[2]
        self.pose.orientation.w = self.q[3]
        print(self.pose)
        self.arm.move(self.pose)

    def pouch_planning_vertical(self, trans, right_angle):
        print("start setting")
        tr = trans
        self.pose = geometry_msgs.msg.Pose()
        #直角か否か
        if right_angle == True:
            self.q = tf.transformations.quaternion_from_euler(0, 0, math.pi/2)
        else:
            self.q = tf.transformations.quaternion_from_euler(0, 0, math.pi)
        self.pose.position.x = tr[0]-0.03
        self.pose.position.y = tr[1]-0.04
        self.pose.position.z = tr[2]+0.08 # debug
        self.pose.orientation.x = self.q[0]
        self.pose.orientation.y = self.q[1]
        self.pose.orientation.z = self.q[2]
        self.pose.orientation.w = self.q[3]
        print(self.pose)
        self.arm.move(self.pose)
        
        self.pose.position.x = tr[0]-0.03
        self.pose.position.y = tr[1]-0.04
        self.pose.position.z = tr[2]-0.04 # debug
        self.pose.orientation.x = self.q[0]
        self.pose.orientation.y = self.q[1]
        self.pose.orientation.z = self.q[2]
        self.pose.orientation.w = self.q[3]
        print(self.pose)
        self.arm.move(self.pose)


    def gripping_vertical(self, ids, right_angle):
        """
        @brief 吸着可能な面がない場合の把持動作関数
        planning_vertical にboolの right_angle が送られて把持角度の選択が可能

        """
        exist = 0
        rospy.sleep(2)
        # ar_track topic
        print(self.ar_info)
        for data in self.ar_info:
            if data.id in ids:
                grab_id = int(data.id)
                exist = 1
                break
            else:
                continue
        if exist == 1:
            self.ar_name = "ar_marker_" + str(grab_id)
            self.listener.waitForTransform("/camera_link", self.ar_name, rospy.Time(), rospy.Duration(7.0))
            (trans,rot) = self.listener.lookupTransform('/world', '/%s' % (self.ar_name), rospy.Time(0))
            self.planning_vertical(trans, right_angle)


    def pouch_gripping_vertical(self, ids, right_angle):
        """
        @brief 吸着可能な面がない場合の把持動作関数
        planning_vertical にboolの right_angle が送られて把持角度の選択が可能

        """
        exist = 0
        rospy.sleep(2)
        # ar_track topic
        print(self.ar_info)
        for data in self.ar_info:
            if data.id in ids:
                grab_id = int(data.id)
                exist = 1
                break
            else:
                continue
        if exist == 1:
            self.ar_name = "ar_marker_" + str(grab_id)
            self.listener.waitForTransform("/camera_link", self.ar_name, rospy.Time(), rospy.Duration(7.0))
            (trans,rot) = self.listener.lookupTransform('/world', '/%s' % (self.ar_name), rospy.Time(0))
            trans[2] = trans[2] + 0.02 
            self.pouch_planning_vertical(trans, right_angle)


    def sand_gripping_vertical(self, ids, right_angle):
        """
        @brief 吸着可能な面がない場合の把持動作関数
        planning_vertical にboolの right_angle が送られて把持角度の選択が可能

        """
        exist = 0
        rospy.sleep(2)
        # ar_track topic
        print(self.ar_info)
        for data in self.ar_info:
            if data.id in ids:
                grab_id = int(data.id)
                exist = 1
                break
            else:
                continue
        if exist == 1:
            self.ar_name = "ar_marker_" + str(grab_id)
            self.listener.waitForTransform("/camera_link", self.ar_name, rospy.Time(), rospy.Duration(7.0))
            (trans,rot) = self.listener.lookupTransform('/world', '/%s' % (self.ar_name), rospy.Time(0))
            trans[0] = trans[0] + 0.03
            trans[2] = trans[2] + 0.04 
            self.planning_vertical(trans, right_angle)

    
    def vertical_display(self, trans, right_angle):
        print("start setting")
        tr = trans
        self.pose = geometry_msgs.msg.Pose()
        #直角か否か
        if right_angle == True:
            self.q = tf.transformations.quaternion_from_euler(0, 0, math.pi/2)
        else:
            self.q = tf.transformations.quaternion_from_euler(0, 0, math.pi)
        self.pose.position.x = tr[0]
        self.pose.position.y = tr[1]
        self.pose.position.z = tr[2]
        self.pose.orientation.x = self.q[0]
        self.pose.orientation.y = self.q[1]
        self.pose.orientation.z = self.q[2]
        self.pose.orientation.w = self.q[3]
        print(self.pose)
        self.arm.move(self.pose)

        self.pose.position.x = tr[0]
        self.pose.position.y = tr[1]
        self.pose.position.z = tr[2] 
        self.pose.orientation.x = self.q[0]
        self.pose.orientation.y = self.q[1]
        self.pose.orientation.z = self.q[2]
        self.pose.orientation.w = self.q[3]
        print(self.pose)
        self.arm.move(self.pose)



    
    def display(self, ar_id, task):
        # ifで分類
        #指定座標に向かい吸着
        if ar_id == "init":
            self.vertical_display([-0.0, -0.36, -0], False)
        if ar_id == "ABC" :
            if task == "pick":
                self.vertical_display([-0.45, -0.3, -0.0], True)
                self.vertical_display([-0.45, -0.3, -0.17], True)
            elif task == "place":
                self.vertical_display([0, -0.3, 0.5], True)
                self.vertical_display([0.26, 0.2, 0.47], True)

        if ar_id == "DEF":
            if task == "pick":
                self.vertical_display([-0.35, -0.3, -0.0], True)
                self.vertical_display([-0.35, -0.3, -0.17], True)
            elif task == "place":
                self.vertical_display([0, -0.3, 0.5], True)
                self.vertical_display([0.26, 0.0, 0.47], True)



if __name__ == "__main__":
    rospy.init_node('shelf', anonymous=True)
    robot = XArm_command()
    
    try:
        #robot.display(None)
        
        #robot.look_shelf("high", "right")
        #robot.look_shelf("middle", "left")
        robot.look_shelf("middle", "right")
        #rospy.sleep(4)
        #robot.display("A", "pick")
        #robot.display("A", "place")
        
        #robot.look_shelf("low", "right")

	#ids = range(37, 42)
        #robot.gripping_vertical(ids, True)
        #flag = robot.exist_ar(ids)
        #print(flag)
        #robot.get_optimul_ar(ids)
        #robot.look_shelf("high", "left")
        #robot.vacuum_picking(1)
        #robot.waste_high_objects()
        #robot.look_shelf("high", "right")
        #robot.gripper.vacuum_on()
        #robot.bring_to_box()
        #rospy.sleep(1)
        #robot.gripper.vacuum_off()
        #print("vacuum end")
        #rospy.spin()
	
    except rospy.ROSInterruptException:
        pass


