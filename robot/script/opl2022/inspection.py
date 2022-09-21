#!/usr/bin/env python
# cooding: utf-8

import time
import sys

import rospy
from std_msgs.msg import String
import smach
import smach_ros

sys.path.append('/home/demulab/opl_ws/src/OPL22/robot/script')
from module import *



def main():
    rospy.init_node("main", anonymous=True)
    voice = Voice()
    move = Move()
    voice.play("start_inspection.mp3")
    voice.play("please_open_the_door.mp3")
    move.navigation('entry')
    move.navigation('dining')
    move.navigation('exit')
    voice.play("success_inspection.mp3")

if __name__ == '__main__':
    try:
        main() 
    except rospy.ROSInterruptException:
        pass
