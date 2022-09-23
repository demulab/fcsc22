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
    B
    rospy.sleep(1.0)
    self.move.x_move(-1.25)
    rospy.sleep(2.0)

def move_goal():
    rospy.sleep(2.0)
    self.move.x_move(1.25)
    rospy.sleep(1.0)
    self.move.y_move(-1.4)
    rospy.sleep(2.0)

class ARPick(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['success'])

    def execute(self,userdata):
        return 'success'

class Pick(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['success'])

    def execute(sefl,userdata):

        return 'success'

class LowShelf(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['success'])

    def execute(self,userdata):
        return 'success'

class MiddleShelf(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['success'])

    def execute(self,userdata):
        return 'success'


def main():
    #arm = Arm()
    #gripper = Gripper()
    #vision = Vision()
   # waste_id = get_id()
   # move_shelf()
    sm_low = smach.StateMachine(outcomes=['to_middle_shelf'])
    with sm_low:
        smach.StateMachine.add('LOWSHELF',LowShelf(),transitions={'success':'PICK_coffee'})
        smach.StateMachine.add('PICK_coffee',ARPick(),transitions={'success':'to_middle_shelf'})            
    sm_middle = smach.StateMachine(outcomes=['to_exit'])
    with sm_middle:
        smach.StateMachine.add('MIDDLESHLF',MiddleShelf(),transitions={'success':'PICK_egg_sand'})
        smach.StateMachine.add('PICK_egg_sand',Pick(),transitions={'success':'PICK_lettuce_sand'})
        smach.StateMachine.add('PICK_lettuce_sand',Pick(),transitions={'success':'to_exit'})
    #move_goal()
    sm_top =smach.StateMachine(outcomes=['success'])
    with sm_top:
        smach.StateMachine.add('LOW',sm_low,transitions={'to_middle_shelf':'MIDDLE'})
        smach.StateMachine.add('MIDDLE',sm_middle,transitions={'to_exit':'success'})
    
    outcomes = sm_top.execute()

"""
    sm_top = StateMachine(outcomes=['to_exit']
        with sm_top:
            smach.StateMachine.add('
"""

if __name__ == '__main__':
    try:
        rospy.init_node('fcsc2022', anonymous = True)
        main()
    except rospy.ROSInterruptException:
        pass
