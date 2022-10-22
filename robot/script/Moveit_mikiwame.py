#! /usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import rospkg
from gazebo_msgs.srv import SpawnModel,DeleteModel
from geometry_msgs.msg import PoseStamped,Pose,Point,Quaternion
sys.path.append('/home/demulab/opl_ws/src/fcsc22/robot/script')
from module import *
from manipulation import *

x_axis_var = - 0.18 # 元の位置からの距離
back_x_axis_var = 0.10
class Spawner(Manipulation):
    
    def __init__(self):
        print("Spawner Initial")
        super(Spawner, self).__init__()
        self.x_error = -0.1 
        rospy.sleep(2)  # Waiting for PlanningSceneInterface

    def shelf_base_mesh(self, x, y, z, qx, qy, qz, qw):# shelf_base_add
    # Add a mesh to the planning scene
        rospy.sleep(2)  # Waiting for PlanningSceneInterface
        s_pose = PoseStamped()
        s_pose.header.frame_id = self.group.get_planning_frame()
        s_pose.pose.position.x = x
        s_pose.pose.position.y = y
        s_pose.pose.position.z = z
        s_pose.pose.orientation.x = qx
        s_pose.pose.orientation.y = qy
        s_pose.pose.orientation.z = qz
        s_pose.pose.orientation.w = qw
        self.scene.add_mesh("shelf_base", s_pose, "/home/demulab/opl_ws/src/fcsc22/robot/stl/base_link.stl",(0.001,0.001,0.001))

    def mikiwame_base(self):
        s_pose = PoseStamped()
        s_pose.header.frame_id = self.group.get_planning_frame()
        s_pose.pose.position.x = 1 + self.x_error + x_axis_var
        s_pose.pose.position.y = 0
        s_pose.pose.position.z = -0.555 -0.32 + 0.08
        s_pose.pose.orientation.x = 0
        s_pose.pose.orientation.y = 0
        s_pose.pose.orientation.z = -1
        s_pose.pose.orientation.w = 1
        self.scene.add_mesh("shelf_base", s_pose, "/home/demulab/opl_ws/src/fcsc22/robot/stl/base_link.stl",(0.001,0.001,0.001))
        #self.call_other_boards()

    def back_mikiwame_base(self):
        s_pose = PoseStamped()
        s_pose.header.frame_id = self.group.get_planning_frame()
        s_pose.pose.position.x = 1 + self.x_error + x_axis_var + back_x_axis_var
        s_pose.pose.position.y = 0
        s_pose.pose.position.z = -0.555 -0.32 + 0.08
        s_pose.pose.orientation.x = 0
        s_pose.pose.orientation.y = 0
        s_pose.pose.orientation.z = -1
        s_pose.pose.orientation.w = 1
        self.scene.add_mesh("shelf_base", s_pose, "/home/demulab/opl_ws/src/fcsc22/robot/stl/base_link.stl",(0.001,0.001,0.001))
        #self.call_other_boards()


    def shelf_board_mesh(self, board_id, x, y, z, qx, qy, qz, qw):# shelf_board_add
        # Add a mesh to the planning scene
        rospy.sleep(2)  # Waiting for PlanningSceneInterface
        s_pose = PoseStamped()
        s_pose.header.frame_id = self.group.get_planning_frame()
        s_pose.pose.position.x = x + self.x_error
        s_pose.pose.position.y = y
        s_pose.pose.position.z = z
        s_pose.pose.orientation.x = qx
        s_pose.pose.orientation.y = qy
        s_pose.pose.orientation.z = qz
        s_pose.pose.orientation.w = qw
        board_name = "shelf_board_" + str(board_id)
        board_path = os.path.join("/home/demulab/opl_ws/src/fcsc22/robot/stl","Link1_" + str(board_id)+".stl" )
        self.scene.add_mesh(board_name, s_pose, board_path, (0.001,0.001,0.001))

    def remove_object(self, object):
        rospy.sleep(2)
        self.scene.remove_world_object(object)

    def remove_world(self):
        rospy.sleep(2)
        self.scene.remove_world_object()
    
    def call_other_boards(self):
        for i in range(2, 7):
            self.shelf_board_mesh(i, 1 + x_axis_var , 0, -0.555 -0.32 - (0.175 * (i-2)) + 0.08, 0, 0, -1, 1)

    def back_call_other_boards(self):
        for i in range(2, 7):
            self.shelf_board_mesh(i, 1 + x_axis_var + back_x_axis_var , 0, -0.555 -0.32 - (0.175 * (i-2)) + 0.08, 0, 0, -1, 1)

    def open_shelf(self, step):
        if step == "low":
            self.back_mikiwame_base()
            self.remove_object("shelf_board_1")
            self.shelf_board_mesh(1, 0.7 + x_axis_var , 0, -0.555 -0.32 -(0.35*2) + 0.08, 0, 0, -1, 1)
            self.back_call_other_boards()
        elif step == "middle":
            self.back_mikiwame_base()
            self.remove_object("shelf_board_1")
            self.shelf_board_mesh(1, 0.7 + x_axis_var + back_x_axis_var , 0, -0.555 -0.32 - 0.35 + 0.08, 0, 0, -1, 1)
            self.back_call_other_boards()
        elif step == "high":
            self.mikiwame_base()
            self.remove_object("shelf_board_1")
            self.shelf_board_mesh(1, 0.7 + x_axis_var , 0, -0.555 -0.32 + 0.08, 0, 0, -1, 1)
            

        else:
            pass

    def spawn_shelf_low(self):
        print("start")
        self.open_shelf("low")
        rospy.sleep(10)
        rospy.sleep(5)

def main():
    
    print("start")
    rospy.init_node("shelf_moveit", anonymous=True)

    spawner = Spawner()
    #spawner.mikiwame_base()
    print("low")
    spawner.open_shelf("low")
    rospy.sleep(10)
    print("middle")
    spawner.open_shelf("middle")
    rospy.sleep(10)
    print("high")
    spawner.open_shelf("high")
    rospy.sleep(10)
    spawner.remove_world()

    """
    spawner.shelf_base_mesh(1, 0, -0.555, 0, 0, -1, 1)
    spawner.shelf_board_mesh(1, 0, -0.555, 0, 0, -1, 1)
    spawner.open_shelf("low")
    rospy.sleep(10)
    spawner.open_shelf("middle")
    rospy.sleep(5)
    spawner.open_shelf("high")
    spawner.shelf_board_mesh(0.7, 0, -0.555, 0, 0, -1, 1)
    rospy.sleep(5)
    spawner.remove_object("shelf_base")
    spawner.remove_world()
    """
if __name__ == "__main__":
    try:
        main()

    except rospy.ROSInterruptException:
        pass

