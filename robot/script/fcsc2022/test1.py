#!/usr/bin/env python
# cooding: utf-8

import rospy
import smach
import smach_ros
import sys

sys.path.append('/home/demulab/opl_ws/src/fcsc22/robot/script')
from module import *
import rosparam 
import os

# global value
object_id = {"a":1,"b":2,"c":3,"d":4,"e":5,
             "f":6,"g":7,"h":8,"i":9,"j":10,
             "k":11,"l":12,"m":13,"n":14,"o":15,
             "p":16,"q":17,"r":18,"s":19,"t":20,
             "u":21,"v":22,"w":23,"x":24,"y":25,
             "z":26}
waste_id =[0,0,0,0,0,0,0,0,0]

def get_id():
    #Specify the object to be discarded as a string
    input_data = raw_input('please str data: ')
    discarded = input_data.split(',')
    for i in range(len(discarded)):
        waste_id[i] = object_id.get(discarded[i])
    return waste_id

def move_shelf():
    rospy.sleep(2.0)
    self.move.y_move(1.4)
    rospy.sleep(1.0)
    self.move.x_move(-1.25)
    rospy.sleep(2.0)

def move_goal():
    rospy.sleep(2.0)
    self.move.x_move(1.25)
    rospy.sleep(1.0)
    self.move.y_move(-1.4)
    rospy.sleep(2.0)

def main():
    arm = Arm()
    gripper = Gripper()
    vision = Vision()
    waste_id = get_id()
    sm_top = smach.StateMachine(outcomes = ['FINISH'])
    with sm_top:
        sm

if __name__ == '__main__':
    try:
        rospy.init_node('fcsc2022', anonymous = True)
        main()
    except rospy.ROSInterruptException:
        pass
