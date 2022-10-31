#!/usr/bin/env python
# coding: utf-8

import rospy
import geometry_msgs.msg
import tf
import math
import actionlib
from geometry_msgs.msg import Pose
# Arm
import manipulation
from xarm.msg import MoveAction
# Gripper
from xarm_gripper.msg import GripperAction
# Vision
from bounding_to_position.msg import object_pose
from std_msgs.msg import Float32MultiArray
# Voice
from gtts import gTTS
from mutagen.mp3 import MP3 as mp3
import pygame
import time
# Move 
import rosparam
import os
import sys
from std_msgs.msg import String, Float64, Bool
from std_srvs.srv import Empty
from xarm_msgs.srv import SetInt16
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
sys.path.append('/home/demulab/opl_ws/src/fcsc22/scout_ros/scout_navigation/srv')
from scout_navigation.srv import NaviLocation, NaviLocationResponse
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import subprocess

class Arm():
    def __init__(self):
        self.move_act = actionlib.SimpleActionClient('xarm6_moveit', MoveAction) 
        self.move_goal = MoveAction().action_goal.goal
        self.plan_success = False
        self.mani = manipulation.Manipulation()

    def init_pose(self):
        self.move_act.wait_for_server(rospy.Duration(10)) 
        self.move_goal.mode = True
        self.move_act.send_goal(self.move_goal, feedback_cb = self.plan_cb)
        move_result = self.move_act.wait_for_result(rospy.Duration(10))
        return move_result
       
    def move(self, target_pose):
        self.move_act.wait_for_server(rospy.Duration(10)) 
        self.move_goal.mode = False
        self.move_goal.target_pose = target_pose
        self.move_act.send_goal(self.move_goal, feedback_cb = self.plan_cb)
        move_result = self.move_act.wait_for_result(rospy.Duration(10))
        return move_result 

    def add_table(self):
        self.mani.addTable()

    def stop(self):
        self.move_act.cancel_goal()

    def plan_cb(self, msg):
        print("plan_feedback", msg.plan)
        self.plan_success = msg.plan

class Gripper():
    def __init__(self):
        self.gripper_act = actionlib.SimpleActionClient('xarm/gripper_move', GripperAction)
        self.gripper_goal = GripperAction().action_goal.goal
        self.speed = 1200
        rospy.Subscriber("/puressure", Float32MultiArray, self.puressure_cb)
        self.puressure_right = self.puressure_left = 0
        self.gripper_vacuum =rospy.ServiceProxy('/xarm/vacuum_gripper_set',SetInt16)

    def set_speed(self, speed):
        self.speed = speed
   
    def open(self):
        self.gripper_act.wait_for_server(rospy.Duration(10))
        pos = 850
        self.gripper_goal.target_pulse = pos
        self.gripper_goal.pulse_speed = self.speed
        self.gripper_act.send_goal(self.gripper_goal, feedback_cb = self.gripper_cb)
        gripper_result = self.gripper_act.wait_for_result(rospy.Duration(3))
        return gripper_result
 
    def close(self): 
        self.gripper_act.wait_for_server(rospy.Duration(10))
        pos = 0
        self.gripper_goal.target_pulse = pos
        self.gripper_goal.pulse_speed = self.speed
        self.gripper_act.send_goal(self.gripper_goal, feedback_cb = self.gripper_cb)
        gripper_result = self.gripper_act.wait_for_result(rospy.Duration(3))
        return gripper_result

    def stop(self):
        self.gripper_act.cancel_goal()
 
    def gripper_cb(self, msg):
        print("gripper_feedback", msg.current_pulse)

    def puressure_cb(self, msg):
        self.puressure_right = msg.data[0]
        self.puressure_left = msg.data[1] 

    def vacuum_on(self):
        rospy.wait_for_service('/xarm/vacuum_gripper_set')
        self.gripper_vacuum(1)

    def vacuum_off(self):
        rospy.wait_for_service('/xarm/vacuum_gripper_set')
        self.gripper_vacuum(0)

class Vision():
    def __init__(self):
        rospy.Subscriber("/object_pose", object_pose, self.target_object_cb)
        self.target_object = object_pose()
        self.update_flag = False
        self.update_flag_list = []
        self.update_check_num = 10
        self.update = False
    
    def target_object_cb(self, msg): 
        self.target_object = msg
        self.target_object.pose.pose.orientation.x = 0
        self.target_object.pose.pose.orientation.y = 0
        self.target_object.pose.pose.orientation.z = 0
        self.target_object.pose.pose.orientation.w = 0

        # print(self.target_object)
        if self.update_flag == False:
            self.update_flag = True
        else:
            self.update_flag = False

    def check_update(self):
        self.update_flag_list.append(self.update_flag)
        if len(self.update_flag_list) == self.update_check_num:
            sum_num = sum(self.update_flag_list)
            del self.update_flag_list[:]
            if sum_num == 0 or sum_num == self.update_check_num: 
                self.update = False
            else: 
                self.update = True
                 
class Voice():
    def __init__(self):
        self.path = "/home/demulab/opl_ws/src/OPL22/robot/mp3/"
        pygame.mixer.init()
    
    def text_to_mp3(self, text, filename):
        tts = gTTS(text, lang="en", slow=True)
        tts.save(self.path + filename)

    def play(self, filename):
        pygame.mixer.music.load(self.path + filename)
        mp3_length = mp3(self.path + filename).info.length
        pygame.mixer.music.play(1)
        time.sleep(mp3_length)
        pygame.mixer.music.stop() 

    """"def text_play(self, text):
        self.text_to_mp3(text)
        self.play()"""

class Move():
    def __init__(self):
        self.location_list = ['living','tableA','tableB','dining','shelf','exit','restart']
        #self.pub_twist = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)
        self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.twist_value = Twist()    
        self.sub_switch = rospy.Subscriber("/switch", Bool , self.callback)
        self.ac = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    def callback(self,msg):
         print(bool(msg))
         if bool(msg) == True:
             self.twist_value.linear.x = 0.0
             self.twist_value.linear.y = 0.0
             self.twist_value.angular.z = 0.0
             self.ac.cancel_goal()
             self.pub_twist.publish(self.twist_value)

    def stop(self):
        self.twist_value.linear.x = 0.0
        self.twist_value.linear.y = 0.0
        self.twist_value.angular.z = 0.0
        self.pub_twist.publish(self.twist_value)
         
    def x_move(self, distance):
        target_time = abs(distance / 0.15)
        if distance >0:
            self.twist_value.linear.x = 0.23
        elif distance < 0:
            self.twist_value.linear.x = -0.23
        rate = rospy.Rate(500)
        start_time = time.time()
        end_time = time.time()
        while end_time - start_time <= target_time:
            self.pub_twist.publish(self.twist_value)
            end_time = time.time()
            rate.sleep()
        self.twist_value.linear.x = 0.0
        self.pub_twist.publish(self.twist_value)

    def y_move(self, distance):
        target_time = abs(distance / 0.15)
        if distance >0:
            self.twist_value.linear.y = 0.23
        elif distance < 0:
            self.twist_value.linear.y = -0.23
        rate = rospy.Rate(500)
        start_time = time.time()
        end_time = time.time()
        while end_time - start_time <= target_time:
            self.pub_twist.publish(self.twist_value)
            end_time = time.time()
            rate.sleep()
        self.twist_value.linear.y = 0.0
        self.pub_twist.publish(self.twist_value)

    def move_angle(self, degree):
        while degree > 180:
            degree = degree - 360
        while degree < -180:
            degree = degree + 360
        angular_speed = 45 #[deg/s]
        target_time = abs(degree /angular_speed) #[s]
        if degree >= 0:
            self.twist_value.angular.z = (angular_speed * 3.14159263 / 180.0) #rad
        elif degree < 0:
            self.twist_value.angular.z = -(angular_speed * 3.14159263 / 180.0) #rad
        rate = rospy.Rate(1000)
        start_time = time.time()
        end_time = time.time()
        while end_time - start_time <= target_time:
            self.pub_twist.publish(self.twist_value)
            end_time = time.time()
            rate.sleep()
        self.twist_value.angular.z = 0.0
        self.pub_twist.publish(self.twist_value)

    def navigation(self,target_name):
        location_dict = rosparam.get_param('/location')
        if target_name in location_dict:
            coord_list = location_dict[target_name]
            rospy.loginfo("Start Navigation")

            self.ac.wait_for_server()
            clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = coord_list[0]
            goal.target_pose.pose.position.y = coord_list[1]
            goal.target_pose.pose.orientation.z = coord_list[2]
            goal.target_pose.pose.orientation.w = coord_list[3]

            clear_costmaps()
            rospy.wait_for_service('move_base/clear_costmaps')
            rospy.sleep(0.3)
            self.ac.send_goal(goal)
            state = self.ac.get_state()
            count = 0 # clear_costmapsの実行回数をカウンタ
            clear = 0
            while not rospy.is_shutdown():
                state = self.ac.get_state()
                if state == 1:
                    rospy.loginfo('Running...')
                    rospy.sleep(1.0)
                    clear += 1
                elif state == 3:
                    rospy.loginfo('Navigation success!!')
                    state = 0
                    return True
                elif state == 4:
                     if count == 3:
                         count = 0
                         rospy.loginfo('Navigation Failed')
                         return False
                     else:
                         rospy.loginfo('Clearing Costmaps')
                         clear_costmaps()
                         self.ac.send_goal(goal)
                         rospy.loginfo('Send Goal')
                     count += 1

                if clear == 5:
                    clear_costmaps()
                    rospy.loginfo('Clearing Costmaps')
                    clear = 0
        else:
            return False
####################################################################################################################
##################################################### FCSC 2022 ####################################################
####################################################################################################################
def ShelfCommand(shelf_command):
    print("start")
    if shelf_command == "high_open":
        print("open")
        for i in range(0,2):
            res = subprocess.call(["python","/home/demulab/opl_ws/src/fcsc22/robot/script/irmcli.py","-p","-f","/home/demulab/opl_ws/src/fcsc22/robot/json/open_high_shelf.json"])
        if res == 0:
            print("high_open")
    elif shelf_command == "high_close":
        for i in range(0,2):
            res = subprocess.call(["python","/home/demulab/opl_ws/src/fcsc22/robot/script/irmcli.py","-p","-f","/home/demulab/opl_ws/src/fcsc22/robot/json/close_high_shelf.json"])
        if res == 0:
            print("high_close")
    elif shelf_command == "middle_open":
        for i in range(0,2):
            res = subprocess.call(["python","/home/demulab/opl_ws/src/fcsc22/robot/script/irmcli.py","-p","-f","/home/demulab/opl_ws/src/fcsc22/robot/json/open_middle_shelf.json"])
        if res == 0:
            print("middle_open")
    elif shelf_command == "middle_close":
        for i in range(0,2):
            res = subprocess.call(["python","/home/demulab/opl_ws/src/fcsc22/robot/script/irmcli.py","-p","-f","/home/demulab/opl_ws/src/fcsc22/robot/json/close_middle_shelf.json"])
        if res == 0:
            print("middle_open")
    elif shelf_command == "low_open":
        for i in range(0,2):
            res = subprocess.call(["python","/home/demulab/opl_ws/src/fcsc22/robot/script/irmcli.py","-p","-f","/home/demulab/opl_ws/src/fcsc22/robot/json/open_low_shelf.json"])
        if res == 0:
            print("low_open")
    elif shelf_command == "low_close":
        for i in range(0,3):
            res = subprocess.call(["python","/home/demulab/opl_ws/src/fcsc22/robot/script/irmcli.py","-p","-f","/home/demulab/opl_ws/src/fcsc22/robot/json/close_low_shelf.json"])
        if res == 0:
            print("low_close")
        print("irmagician_acitive activate")
