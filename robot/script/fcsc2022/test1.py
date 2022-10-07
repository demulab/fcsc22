#!/usr/bin/env python
# cooding: utf-8

import rospy
import smach
import smach_ros
import sys

sys.path.append('/home/demulab/opl_ws/src/fcsc22/robot/script')
from module import *
from fcsc_command import *
from Moveit_mikiwame import *
import rosparam 
import os

# global value
object_id = {"a":1,"b":2,"c":3,"d":4,"e":5,
             "f":6,"g":7,"h":8,"i":9,"j":10,
             "k":11,"l":12,"m":13,"n":14,"o":15,
             "p":16,"q":17,"r":18,"s":19,"t":20,
             "u":21,"v":22,"w":23,"x":24,"y":25,
             "z":26}
coffee_id =[1,2,3]
bento_id = [4,5,6]
pouch_id = [7,8,9]
egg_sandwich_id = [10,11,12]
lettuce_sandwich_id = [13,14,15]
onigiri_syake_id = [16,17,18]
onigiri_ume_id = [19,20,21]
onigiri_tuna_id = [22,23,24]
unnone_id = [25,26,27]

coffee_waste = 0
bento_waste = 0
pouch_waste = 0
egg_sandwich_waste = 0
lettuce_sandwich_waste = 0
onigiri_syake_waste = 0
onigiri_ume_waste = 0
onigiri_tuna_waste = 0
unnone_waste = 0
#wasted_id = [
move = Move()

def get_id():
    #Specify the object to be discarded as a string
    waste_id = [0,0,0,0,0,0,0,0,0]
    input_data = raw_input('please str data: ')
    discarded = input_data.split(',')
    for i in range(len(discarded)):
        waste_id[i] = object_id.get(discarded[i])
    #print(waste_id)
    for i in range(len(coffee_id)):
        if coffee_id[i] in waste_id:
            coffee_waste = coffee_id[i]
    
    for i in range(len(bento_id)):
        if bento_id[i] in waste_id:
            bento_waste = bento_id[i]

    for i in range(len(pouch_id)):
        if pouch_id[i] in waste_id:
            pouch_waste = pouch_id[i]
 
    for i in range(len(egg_sandwich_id)):
        if egg_sandwich_id[i] in waste_id:
            egg_sandwich_waste = egg_sandwich_id[i]

    for i in range(len(lettuce_sandwich_id)):
        if lettuce_sandwich_id[i] in waste_id:
            lettuce_sandwich_waste = lettuce_sandwich_id[i]
      
    for i in range(len(onigiri_syake_id)):
        if onigiri_syake_id[i] in waste_id:
            onigiri_syake_waste = onigiri_syake_id[i]

    for i in range(len(onigiri_ume_id)):
        if onigiri_ume_id[i] in waste_id:
            onigiri_ume_waste = onigiri_ume_id[i]
     
    for i in range(len(onigiri_tuna_id)):
        if onigiri_tuna_id[i] in waste_id:
            onigiri_tuna_waste = onigiri_tuna_id[i]

    for i in range(len(unnone_id)):
        if unnone_id[i] in waste_id:
            unnone_waste = unnone_id[i]

    return coffee_waste
            

def move_shelf():
    rospy.sleep(2.0)
    move.y_move(1.25)
    rospy.sleep(1.0)
    move.x_move(-1.20)
    rospy.sleep(2.0)

def move_goal():
    rospy.sleep(2.0)
    self.move.x_move(-1.20)
    rospy.sleep(1.0)
    self.move.y_move(-2.5)
    rospy.sleep(2.0)


class LowShelf(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['success'])

    def execute(self,userdata):
        print(coffee_waste)
       # id_data = get_id()
        
        print(id_data)
        command = XArm_command()
        spawner = Spawner()
        command.look_shelf("low","left")
        rospy.sleep(3.0)
        
        spawner.remove_world()
        spawner.mikiwame_base()
        spawner.spawn_shelf_low()
        ShelfCommand("low_open")
        rospy.sleep(4.0)
        command.ar_picking(1)
        command.look_shelf("low","left")

        #to contena
        
        #x_command.look_shlef(low,right)

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
    #spawner = Spawner()
    get_id()
    #move_shelf()
    command = XArm_command()

    sm_low = smach.StateMachine(outcomes=['to_middle_shelf'])
    with sm_low:
        smach.StateMachine.add('LOWSHELF',LowShelf(),transitions={'success':'to_middle_shelf'})
    sm_middle = smach.StateMachine(outcomes=['to_exit'])
    with sm_middle:
        smach.StateMachine.add('MIDDLESHLF',MiddleShelf(),transitions={'success':'to_exit'})
    #move_goal()
    sm_top =smach.StateMachine(outcomes=['success'])
    with sm_top:
        smach.StateMachine.add('LOW',sm_low,transitions={'to_middle_shelf':'MIDDLE'})
        smach.StateMachine.add('MIDDLE',sm_middle,transitions={'to_exit':'success'})
    
    outcomes = sm_top.execute()
    #move_goal()

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
