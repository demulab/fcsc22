#!/usr/bin/env python
# cooding: utf-8
# 

import rospy
import smach
import smach_ros
import sys
from std_msgs.msg import Bool
sys.path.append('/home/demulab/opl_ws/src/fcsc22/robot/script')
from module import *
from fcsc_command_v3 import *
from Moveit_mikiwame import *
import rosparam 
import os



# global value
object_id = {"a":1,"b":2,"c":3,"d":4,"e":5,
             "f":6,"g":7,"h":8,"i":9,"j":10,
             "k":11,"l":12,"m":13,"n":14,"o":15,
             "p":16,"q":17,"r":18,"s":19,"t":20,
             "u":21,"v":22,"w":23,"x":24,"y":25,
             "z":26,"A":27}
onigiri_ume_id = [1,2,3]
onigiri_syake_id = [4,5,6]
onigiri_tuna_id = [7,8,9]
egg_sandwich_id = [10,11,12]
lettuce_sandwich_id = [13,14,15]
pouch_id = [16,17,18]
coffee_id =[19,20,21]
bentoA_id = [22,23,24]
bentoB_id = [25,26,27]

coffee_waste = 0
bentoA_waste = 0
pouch_waste = 0
egg_sandwich_waste = 0
lettuce_sandwich_waste = 0
onigiri_syake_waste = 0
onigiri_ume_waste = 0
onigiri_tuna_waste = 0
bentoB_waste = 0
waste_id = [0,0,0,0,0,0,0,0,0]
#[0:ume, 1:syake, 2:tuan, 3:eggsand, 4:lettuce, 5:pouch , 6:coffee, 7:bentoA, 8:bentoB]
#move = Move()



def get_id():
    #Specify the object to be discarded as a string
    global waste_id    
    input_data = raw_input('please str data: ')
    discarded = input_data.split(',')
    for i in range(len(discarded)):
        waste_id[i] = object_id.get(discarded[i])
    #print(waste_id)
    for i in range(len(coffee_id)):
        if coffee_id[i] in waste_id:
            coffee_waste = coffee_id[i]
    
    for i in range(len(bentoA_id)):
        if bentoA_id[i] in waste_id:
            bentoA_waste = bentoA_id[i]

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

    for i in range(len(bentoB_id)):
        if bentoB_id[i] in waste_id:
            bentoB_waste = bentoB_id[i]

    return coffee_waste
            


def move_shelf():
    print("\n\n\n\n\n\n\n\n\n")
    print("Move start")
    move = Move()
    rospy.sleep(2.0)
    move.y_move(1.25)
    rospy.sleep(1.0)
    move.x_move(-1.20)
    rospy.sleep(2.0)
    print("Move end")
    print("\n\n\n\n\n\n\n\n\n\n\n")
def move_goal():
    move = Move()
    rospy.sleep(2.0)
    move.x_move(1.25)
    rospy.sleep(1.0)
    move.y_move(-1.3)
    rospy.sleep(2.0)


class LowShelf(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['success'])
        self.command = XArm_command()
        self.spawner = Spawner()


    def execute(self,userdata):
        global waste_id
        #id_data = get_id()
        print(waste_id[6])
        
        rospy.sleep(1.0)
        move_shelf()
        self.debug_id = 19
        self.spawner.remove_world()
        # coffee
        if waste_id[6] == 19:
            self.debug_id = 19
        elif waste_id[6] == 20:
            self.debug_id = 20
        elif waste_id[6] == 21:
            self.debug_id == 21
        
        self.command.look_shelf("high","left")
        self.spawner.mikiwame_base()
        self.spawner.open_shelf("high")
        ShelfCommand("high_open")
        rospy.sleep(5.0)
        self.command.vacuum_picking(1)
        self.command.bring_to_box()
        """
        try:
            self.command.vacuum_picking(self.debug_id)#ar_picking(self.debug_id)
            self.command.bring_to_box()
        except:
            pass
        
        self.command.look_shelf("low","left")
        self.spawner.remove_world()
        #to contena
        #x_command.look_shlef(low,right)
        """
        ShelfCommand("high_close")
        """
        #egg sand
        if waste_id[3] == 10:
            self.debug_id = 10
        elif waste_id[3] == 11:
            self.debug_id = 11
        elif waste_id[3] == 12:
            self.debug_id == 12
        self.command.look_shelf("middle", "left")
        self.spawner.mikiwame_base()
        self.spawner.open_shelf("middle")
        ShelfCommand("middle_open")
        rospy.sleep(4)
        try:
            self.command.vacuum_picking(self.debug_id)
            self.command.bring_to_box()
        except:
            pass

        self.command.look_shelf("middle", "left")
        #ShelfCommand("middle_close")
        self.spawner.remove_world()        
        
        #lettuce sand
        if waste_id[4] == 13:
            self.debug_id = 13
        elif waste_id[4] == 14:
            self.debug_id = 14
        elif waste_id[4] == 15:
            self.debug_id = 15
        
        self.command.look_shelf("middle", "left")
        self.spawner.mikiwame_base()
        self.spawner.open_shelf("middle")
        #ShelfCommand("middle_open")
        rospy.sleep(4)
        try:
            self.command.vacuum_picking(self.debug_id)
            self.command.bring_to_box()
        except:
            pass

        self.command.look_shelf("middle","left")
        self.spawner.remove_world()

        # pouch 
        if waste_id[5] == 16:
            self.debug_id = 16
        elif waste_id[6] == 17:
            self.debug_id = 17
        elif waste_id[7] == 18:
            self.debug_id = 18
        self.command.look_shelf("middle", "right")
        self.spawner.mikiwame_base()
        self.spawner.open_shelf("middle")
        #ShelfCommand("middle_open")
        rospy.sleep(4)
        try:
            self.command.vacuum_picking(self.debug_id)
            self.command.bring_to_box()
        except:
            pass

        self.command.look_shelf("middle", "left")
        ShelfCommand("middle_close")
        self.spawner.remove_world()
        """
        


        move_goal()
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
    #get_id()
    #move_shelf()
    #command = XArm_command()

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
