#! /usr/bin/env python

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






class Spawner(Manipulation):
    
    def __init__(self):
        print("Spawner Initial")
        super(Spawner, self).__init__()
	self.x_error = -0.1        

        rospy.sleep(2)  # Waiting for PlanningSceneInterface
       
      #  s_pose = PoseStamped()
       # s_pose.header.frame_id = self.group.get_planning_frame()
       # s_pose.pose.position.x = 1 + self.x_error
       # s_pose.pose.position.y = 0
       # s_pose.pose.position.z = -0.555
       # s_pose.pose.orientation.x = 0
       # s_pose.pose.orientation.y = 0
       # s_pose.pose.orientation.z = -1
       # s_pose.pose.orientation.w = 1
       # self.scene.add_mesh("shelf_base", s_pose, "/home/demulab/opl_ws/src/fcsc22/robot/stl/base_link.stl",(0.001,0.001,0.001))
        


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
        s_pose.pose.position.x = 1 + self.x_error
        s_pose.pose.position.y = 0
        s_pose.pose.position.z = -0.555
        s_pose.pose.orientation.x = 0
        s_pose.pose.orientation.y = 0
        s_pose.pose.orientation.z = -1
        s_pose.pose.orientation.w = 1
        self.scene.add_mesh("shelf_base", s_pose, "/home/demulab/opl_ws/src/fcsc22/robot/stl/base_link.stl",(0.001,0.001,0.001))



    def shelf_board_mesh(self, x, y, z, qx, qy, qz, qw):# shelf_board_add
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
        self.scene.add_mesh("shelf_board_1", s_pose, "/home/demulab/opl_ws/src/fcsc22/robot/stl/Link1_1.stl",(0.001,0.001,0.001))


    def remove_object(self, object):
        rospy.sleep(2)
        self.scene.remove_world_object(object)


    def remove_world(self):
        rospy.sleep(2)
        self.scene.remove_world_object()



    def open_shelf(self, step):

        if step == "low":
            self.remove_object("shelf_board_1")
            self.shelf_board_mesh(0.7 + self.x_error, 0, -0.555 -(0.35*2), 0, 0, -1, 1)
            
        elif step == "middle":
            self.remove_object("shelf_board_1")
            self.shelf_board_mesh(0.7 + self.x_error, 0, -0.555 - 0.35, 0, 0, -1, 1)
        
        elif step == "high":
            self.remove_object("shelf_board_1")
            self.shelf_board_mesh(0.7, 0, -0.555, 0, 0, -1, 1)

        else:
            pass


    def spawn_shelf_low(self):
        print("start")
        #rospy.init_node("shelf_moveit", anonymous=True)

       # spawner = Spawner()
        #spawner.shelf_base_mesh(1, 0, -0.555, 0, 0, -1, 1)
        #spawner.shelf_board_mesh(1, 0, -0.555, 0, 0, -1, 1)
        self.open_shelf("low")
        rospy.sleep(10)
        #spawner.open_shelf("middle")
        #rospy.sleep(5)
        #spawner.open_shelf("high")
        
        #spawner.shelf_board_mesh(0.7, 0, -0.555, 0, 0, -1, 1)
        rospy.sleep(5)
        #spawner.remove_object("shelf_base")
        #spawner.remove_world()
    


"""
#def main():
#    print("start")
#    rospy.init_node("shelf_moveit", anonymous=True)

 #   spawner = Spawner()
    #spawner.shelf_base_mesh(1, 0, -0.555, 0, 0, -1, 1)
    #spawner.shelf_board_mesh(1, 0, -0.555, 0, 0, -1, 1)
 #   spawner.open_shelf("low")
#    rospy.sleep(10)
    #spawner.open_shelf("middle")
    #rospy.sleep(5)
    #spawner.open_shelf("high")
    
    #spawner.shelf_board_mesh(0.7, 0, -0.555, 0, 0, -1, 1)
 #   rospy.sleep(5)
    #spawner.remove_object("shelf_base")
    #spawner.remove_world()

if __name__ == "__main__":
    try:
        main()

    except rospy.ROSInterruptException:
        pass
"""
