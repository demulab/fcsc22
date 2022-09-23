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
sys.path.append('/home/demulab/opl_ws/src/OPL22/robot/script')
from module import *
from manipulation import *






class Spawner(Manipulation):
    
    def __init__(self):
        print("Spawner Initial")
        super(Spawner, self).__init__()

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
        self.scene.add_mesh("shelf_base", s_pose, "base_link.stl",(0.001,0.001,0.001))

    def remove_object(self, object):
        rospy.sleep(2)
        self.scene.remove_world_object(object)

    def remove_world(self):
        rospy.sleep(2)
        self.scene.remove_world_object()


def main():
    print("start")
    rospy.init_node("shelf_moveit", anonymous=True)

    spawner = Spawner()

    spawner.shelf_base_mesh(0, 0.25, 0, 0, 0, 0, 1)
    
    rospy.sleep(30)
    spawner.remove_object("shelf_base")
    

if __name__ == "__main__":
    try:
        main()

    except rospy.ROSInterruptException:
        pass








