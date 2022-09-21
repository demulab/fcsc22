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
    

if __name__ == '__main__':
    arm = Arm()
    gripper = Gripper()
    vision = Vision()
    
    try:
        rospy.init_node('open_campus', anonymous=True)
        main()
    except rospy.ROSInterruptException:
        pass
