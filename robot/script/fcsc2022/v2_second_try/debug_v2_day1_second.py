#!/usr/bin/env python
# cooding: utf-8

import sys
#sys.path.append('/home/demulab/opl_ws/src/fcsc22/robot/script')
#sys.path.append('/home/demulab/opl_ws/src/fcsc22/human_detector_wrs2021/sawyer_human_detector')
import tf
import rospy
import smach
import time
import smach_ros
from std_msgs.msg import Bool, String 
sys.path.append('/home/demulab/opl_ws/src/fcsc22/robot/script')
from module import *
from manipulation import *
from fcsc_command_v3 import *
from Moveit_mikiwame import *
import rosparam 
import os
sys.path.append('/home/demulab/opl_ws/src/fcsc22/human_detector_wrs2021/sawyer_human_detector')
from sawyer_human_detector.msg import Emergency

# global value
object_id = {"a":[1,2,3,4,5,6],"b":[7,8,9,10,11,12],"c":[13,14,15,16,17,18],
             "d":[19,20,21,22,23,24],"e":[25,26,27,28,29,30],"f":[31,32,33,34,35,36],
             "g":[37,38,39,40,41],"h":[42,43,44,45,46],"i":[47,48,49,50,51],
             "j":[52,53,54,55,56],"k":[57,58,59,60,61],"l":[62,63,64,65,66],
             "m":[67,68,69,70,71],"n":[72,73,74,75,76],"o":[77,78,79,80,81],
             "p":[82,83],"q":[84,85],"r":[86,87],"s":[88],"t":[89],
             "u":[90],"v":[91],"w":[92],"x":[93],"y":[94],
             "z":[95],"A":[96]}
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

#state
step = 0
save_step = 0
save_step_flg = False
hoge = False
move = Move()
mani = Manipulation()
spawner = Spawner()
save_time = 0
start_time = 0

def get_id():
    global waste_id
    input_data = raw_input('pleas str data: ')
    discarded = input_data.split(',')
    for i in range(len(discarded)):
        waste_id[i] = object_id.get(discarded[i])
    print(waste_id)


def get_alphabet():
    global save_time
    global start_time
    print("awaiting data")
    datas = raw_input()
    data = list(datas)
    print("giveup_time")
    save_time = input()
    start_time = time.time()
    return datas

#vacuume please Bool
def vacuum(comand):
    vacuum_pub = rospy.Publisher("pickup", Bool, queue_size=10)
    if comand == True or comand == False:
        vacuum_pub.publish(comand)

#pickup please Bool
def pick(comand):
    pick_pub = rospy.Publisher("servo/endeffector",String,queue_size=10)
    if comand == True:
        pick_pub.publish("OPEN")
    elif comand == False:
        pick_pub.publish("CLOSE")

def time_count(x):
    time_step = x + 1
    if (save_time - 30) < time.time() - start_time:
        return 100

    else:
        return time_step

def humanDetectCB(msg):
    global save_step
    global save_step_flg
    global step
    global hoge
    #vacuum(False)
    print("ooooooo")
    print(msg.sawyerstop)
    if msg.sawyerstop ==True:
        print("sub_sawyer_stop")
        print(save_step_flg)
        if save_step_flg == False:
            for i in range(15):
                mani.stop()
                rospy.sleep(0.3)
            vacuum(False)
            save_step = step
            save_step_flg = True
            print("save_step",save_step)
            step = -2
            print("step",step)

""" 
            move.x_move(1.25)
            rospy.sleep(5.0)
            move.x_move(-1.25)
            step = save_step
            print("step",step)
            if 1 <= step <= 3:
                print("low_shelf_open")
                #irmacgisyan
            elif 4 <= step <= 6:
                print("middle_shelf_opne")
            #irmachisin
            elif 7 <= step <= 9:
                print("high_shelf_open")
            #irmagician
"""

def move_shelf():
    print("Move start")
    rospy.sleep(2.0)
    move.y_move(1.15)
    rospy.sleep(1.0)
    move.x_move(-1.30)
    rospy.sleep(2.0)
    print("Move end")

def move_goal():
    rospy.sleep(2.0)
    move.x_move(1.25)
    rospy.sleep(1.0)
    move.y_move(-1.3)
    rospy.sleep(2.0)



def switchCB(msg):
    if bool(msg) == True:
        while not rospy.is_shutdown():
            move.stop()
            print("stop")
            rospy.sleep(1.0)


def main():
    # flags
    retry_flag = False

    #rospy.init_node('main',anonymous=True)
    emergency = Emergency()
    #pub
    emergency_pub = rospy.Publisher('sawyer_to_lidar', Emergency, queue_size=10)
    
    #sub
    rospy.Subscriber("human_detector_command", Emergency,humanDetectCB)
    rospy.Subscriber("switch",Bool,switchCB)

   # global step
   # global hoge
   # global save_step
    r = rospy.Rate(10)
    ip_datas = get_alphabet()    
#debug step 1000
    #step = 1000

    while not rospy.is_shutdown():
        global step
        print('---step=%s--' %(step))

        emergency.mecanum = step
        emergency_pub.publish(emergency)

#debug step =1000
        if step == 1000:
            rospy.sleep(2.0)
            step = time_count(step)


        if step == 0:
            spawner.remove_world()
            command.look_shelf("high","left")
            spawner.mikiwame_base()
            spawner.open_shelf("high")
            #ip_datas =  get_alphabet()
#################################
            #move_shelf()
################################
            #get_id()
            rospy.sleep(2.0)
            step += 1

        elif step == -1:
            print("stop")
            global hoge
            global save_step
           # print(hoge)
            if hoge == False:
                print("aaaaaaaaaa")
                #mani.stop()
                print("save_step")
                
                if 1 <= save_step <= 29:
                    ShelfCommand("high_close")
                    print("haigh_shelf_close")
                    #move.x_move(-1.25)
                    move.x_move(1.25)
                    command.look_shelf("high","left")
                elif 30 <= save_step <= 56:
                    ShelfCommand("middle_close")
                    print("middle_shelf_close")
                    #move.x_move(-1.25)
                    move.x_move(1.25)
                    command.look_shelf("middle","left")

                    #irmachisin
                elif 57 <= save_step <= 75:
                    ShelfCommand("low_close")
                    print("high_shelf_close")
                    #move.x_move(-1.25)
                    move.x_move(1.25)
                    command.look_shelf("low","left")
                   #irmagician
                hoge = True
                step = save_step

                rospy.sleep(8.0)
                #move.x_move(1.25)
                move.x_move(-1.25)
                print("step",step)
                if 1 <= step <= 29:
                    ShelfCommand("high_open")
                    print("high_shelf_open")
                    #irmacgisyan
                elif 30 <= step <= 55:
                    ShelfCommand("middle_open")
                    print("middle_shelf_opne")
                #irmachisin
                elif 56 <= step <= 183:
                    ShelfCommand("low_open")
                    print("low_shelf_open")
             #irmagician

        elif step == -2:
            print("stop")
            global hoge
            global save_step
            print("hoge",hoge)
            print("save_step",save_step)
            if hoge == False:
                print("aaaaaaaaaa")
                mani.stop()
                if 1 <= save_step <= 29:
                    ShelfCommand("high_close")
                    print("haigh_shelf_close")
                    #move.x_move(-1.25)
                    move.x_move(1.25)
                    command.look_shelf("high","left")
                     #irmacgisyanclose
                elif 30 <= save_step <= 55:
                    ShelfCommand("middle_close")
                    print("middle_shelf_close")
                    #move.x_move(-1.25)
                    move.x_move(1.25)
                    command.look_shelf("middle","left")

                     #irmachisin
                elif 56 <= save_step <= 100:
                    ShelfCommand("low_close")
                    print("high_shelf_close")
                    #move.x_move(-1.25)
                    move.x_move(1.25)
                    command.look_shelf("low","left")
                    #irmagician
                hoge = True
                step = save_step

                rospy.sleep(8.0)
                #left
                #move.x_move(1.25)
                move.x_move(-1.25)
                print("step",step)
                if 1 <= step <= 29:
                    ShelfCommand("high_open")
                    print("high_shelf_open")
                     #irmacgisyan
                elif 30 <= step <= 55:
                    ShelfCommand("middle_open")
                    print("middle_shelf_opne")
                 #irmachisin
                elif 56 <= step <= 100:
                    ShelfCommand("low_open")
                    print("low_shelf_open")


        elif step == 1:
            rospy.sleep(4.0)
            step += 1
##########################################################
##################### high ###############################
##########################################################        
        # call planning scene
        elif step == 2:
            if retry_flag == True:
                spawner.remove_world()
                command.look_shelf("high","left")
                spawner.mikiwame_base()
                spawner.open_shelf("high")
            vacuum(False)
            pick(False)
            print(ip_datas)
            ShelfCommand("high_open")
            rospy.sleep(1.0)
            step  = time_count(step)

        # onigiri A 01
        elif step == 3:
            if "A" in ip_datas:
                print("##########\n\nA exec\n\n##########")
                command.look_shelf("high","left")
                ShelfCommand("high_open")
                ids = range(1, 7)
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                print("##########\n\nA passed\n\n##########")
                #####
                step = time_count(step)
                if step != 100:
                    step = 6
                #####

        elif step == 4:
            if flag == False:
                command.look_shelf("high","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)

        elif step == 5:
            if flag == True:
                command.bring_to_box()
                vacuum(False)
                rospy.sleep(3)
            rospy.sleep(1.0)
            step = time_count(step)

        # onigiri B 02
        elif step == 6:
            if "B" in ip_datas:
                print("##########\n\nB exec\n\n##########")
                vacuum(False)
                command.look_shelf("high","left")
                ShelfCommand("high_open")
                ids = range(7, 13)
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                print("###########\n\nB passed\n\n##########")
                step = time_count(step)
                if step != 100:
                    step = 9

        elif step == 7:
            ShelfCommand("high_open")
            if flag == False:
                command.look_shelf("high","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)

        elif step == 8:
            if flag == True:
                command.bring_to_box()
                vacuum(False)
                rospy.sleep(3)
            rospy.sleep(1.0)
            step = time_count(step)
        
        # onigiri C 03
        elif step == 9:
            if "C" in ip_datas:
                print("##########\n\nC exec\n\n##########")
                vacuum(False)
                command.look_shelf("high","left")
                ShelfCommand("high_open")
                ids = range(13, 19)
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                print("##########\n\nC passed\n\n##########")
                step = time_count(step)
                if step != 100:
                    step = 12

        elif step == 10:
            if flag == False:
                command.look_shelf("high","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)

        elif step == 11:
            if flag == True:
                command.bring_to_box()
                vacuum(False)
                rospy.sleep(3)
            rospy.sleep(1.0)
            step = time_count(step)
        
        # onigiri D 04
        elif step == 12:
            if "D" in ip_datas:
                vacuum(False)
                command.look_shelf("high","left")
                ShelfCommand("high_open")
                ids = range(19, 25)
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 15

        elif step == 13:
            if flag == False:
                command.look_shelf("high","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)

        elif step == 14:
            if flag == True:
                command.bring_to_box()
                vacuum(False)
                rospy.sleep(3)
            rospy.sleep(1.0)
            step = time_count(step)

        # onigiri E 05

        elif step == 15:
            if "E" in ip_datas:
                vacuum(False)
                command.look_shelf("high","left")
                ShelfCommand("high_open")
                ids = range(25, 31)
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 18

        elif step == 16:
            if flag == False:
                command.look_shelf("high","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)
        elif step == 17:
            if flag == True:
                command.bring_to_box()
                vacuum(False)
                rospy.sleep(3)
            rospy.sleep(1.0)
            step = time_count(step)

        # onigiri F 06
        elif step == 18:
            if "F" in ip_datas:
                vacuum(False)
                command.look_shelf("high","left")
                ShelfCommand("high_open")
                ids = range(31, 37)
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 21

        elif step == 19:
            if flag == False:
                command.look_shelf("high","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)

        elif step == 20:
            if flag == True:
                command.bring_to_box()
                vacuum(False)
                rospy.sleep(3)
            rospy.sleep(1.0)
            step = time_count(step)
        
        # onigiri G 07
        elif step == 21:
            if "G" in ip_datas:
                pick(False)
                vacuum(False)
                command.look_shelf("high","left")
                ShelfCommand("high_open")
                ids = range(37, 42)
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    if flag == False:
                        vacuum(False)
                        pick(True)
                        command.gripping_vertical(ids, True)
                        flag = True
                        pick(False)
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 24

        elif step == 22:
            if flag == False:
                command.look_shelf("high","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    if flag == False:
                        vacuum(False)
                        pick(True)
                        command.gripping_vertical(ids, True)
                        flag= True
                        pick(False)
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)

        elif step == 23:
            if flag == True:
                command.bring_to_box()
                vacuum(False)
                rospy.sleep(3)
                pick(True)
                rospy.sleep(1)
                pick(False)
            rospy.sleep(1.0)
            step = time_count(step)

        # onigiri H 08



        elif step == 24:
            if "H" in ip_datas:
                pick(False)
                vacuum(False)
                command.look_shelf("high","left")
                ShelfCommand("high_open")
                ids = range(42, 47)
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    if flag == False:
                        vacuum(False)
                        pick(True)
                        command.gripping_vertical(ids, True)
                        flag = True
                        pick(False)
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 27

        elif step == 25:
            if flag == False:
                command.look_shelf("high","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    if flag == False:
                        vacuum(False)
                        pick(True)
                        command.gripping_vertical(ids, True)
                        flag= True
                        pick(False)
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)

        elif step == 26:
            if flag == True:
                command.bring_to_box()
                vacuum(False)
                rospy.sleep(3)
                pick(True)
                rospy.sleep(1)
                pick(False)
            rospy.sleep(1.0)
            step = time_count(step)


        # onigiri I 09
        elif step == 27:
            if "I" in ip_datas:
                pick(False)
                vacuum(False)
                command.look_shelf("high","left")
                ShelfCommand("high_open")
                ids = range(47, 52)
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    if flag == False:
                        vacuum(False)
                        pick(True)
                        command.gripping_vertical(ids, True)
                        flag = True
                        pick(False)
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 30

        elif step == 28:
            if flag == False:
                command.look_shelf("high","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    if flag == False:
                        vacuum(False)
                        pick(True)
                        command.gripping_vertical(ids, True)
                        flag= True
                        pick(False)
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)

        elif step == 29:
            if flag == True:
                command.bring_to_box()
                vacuum(False)
                rospy.sleep(3)
                pick(True)
                rospy.sleep(1)
                pick(False)
            rospy.sleep(1.0)
            step = time_count(step)



##################################
############# middle #############
##################################

        # egg J 10
 
        elif step == 30:
            vacuum(False)
            ShelfCommand("high_close")
            move.y_move(0.2)
            spawner.remove_world()
            spawner.mikiwame_base()
            spawner.open_shelf("middle")
            if "J" in ip_datas:
                pick(False)
                vacuum(False)
                command.look_shelf("middle","left")
                ShelfCommand("middle_open")
                ids = range(52, 57)
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    #if flag == False:
                        #vacuum(False)
                        #pick(True)
                        #command.sand_gripping_vertical(ids, True)
                        #flag = True
                        #pick(False)
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 33

        elif step == 31:
            if flag == False:
                command.look_shelf("middle","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    #if flag == False:
                        #vacuum(False)
                        #pick(True)
                        #command.sand_gripping_vertical(ids, True)
                        #flag= True
                        #pick(False)
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)

        elif step == 32:
            if flag == True:
                command.bring_to_box()
                vacuum(False)
                #rospy.sleep(3)
                #pick(True)
                #rospy.sleep(1)
                #pick(False)
            rospy.sleep(1.0)
            step = time_count(step)


        # egg K 11
        elif step == 33:
            if "K" in ip_datas:
                pick(False)
                vacuum(False)
                command.look_shelf("middle","left")
                ShelfCommand("middle_open")
                ids = range(57, 62)
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    #if flag == False:
                        #vacuum(False)
                        #pick(True)
                        #command.sand_gripping_vertical(ids, True)
                        #flag = True
                        #pick(False)
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 36

        elif step == 34:
            if flag == False:
                command.look_shelf("middle","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    #if flag == False:
                        #vacuum(False)
                        #pick(True)
                        #command.sand_gripping_vertical(ids, True)
                        #flag= True
                        #pick(False)
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)

        elif step == 35:
            if flag == True:
                command.bring_to_box()
                vacuum(False)
                #rospy.sleep(3)
                #pick(True)
                #rospy.sleep(1)
                #pick(False)
            rospy.sleep(1.0)
            step = time_count(step)


        # egg L 12
        elif step == 36:
            if "L" in ip_datas:
                pick(False)
                vacuum(False)
                command.look_shelf("middle","left")
                ShelfCommand("middle_open")
                ids = range(62, 67)
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    #if flag == False:
                        #vacuum(False)
                        #pick(True)
                        #command.sand_gripping_vertical(ids, True)
                        #flag = True
                        #pick(False)
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 39

        elif step == 37:
            if flag == False:
                command.look_shelf("middle","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    #if flag == False:
                        #pick(True)
                        #command.sand_gripping_vertical(ids, True)
                        #flag= True
                        #pick(False)
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)

        elif step == 38:
            if flag == True:
                command.bring_to_box()
                vacuum(False)
                #rospy.sleep(3)
                #pick(True)
                #rospy.sleep(1)
                #pick(False)
            rospy.sleep(1.0)
            step = time_count(step)

        # lettuce M 13


        elif step == 39:
            if "M" in ip_datas:
                pick(False)
                vacuum(False)
                command.look_shelf("middle","left")
                ShelfCommand("middle_open")
                ids = range(67, 72)
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    #if flag == False:
                        #vacuum(False)
                        #pick(True)
                        #command.sand_gripping_vertical(ids, True)
                        #flag = True
                        #pick(False)
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 42

        elif step == 40:
            if flag == False:
                command.look_shelf("middle","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    #if flag == False:
                        #pick(True)
                        #command.sand_gripping_vertical(ids, True)
                        #flag= True
                        #pick(False)
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)

        elif step == 41:
            if flag == True:
                command.bring_to_box()
                vacuum(False)
                #rospy.sleep(3)
                #pick(True)
                #rospy.sleep(1)
                #pick(False)
            rospy.sleep(1.0)
            step = time_count(step)


        # lettuce N 14


        elif step == 42:
            if "N" in ip_datas:
                pick(False)
                vacuum(False)
                command.look_shelf("middle","left")
                ShelfCommand("middle_open")
                ids = range(72, 77)
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    #if flag == False:
                        #vacuum(False)
                        #pick(True)
                        #command.sand_gripping_vertical(ids, True)
                        #flag = True
                        #pick(False)
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 45

        elif step == 43:
            if flag == False:
                command.look_shelf("middle","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    #if flag == False:
                        #pick(True)
                        #command.sand_gripping_vertical(ids, True)
                        #flag= True
                        #pick(False)
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)

        elif step == 44:
            if flag == True:
                command.bring_to_box()
                vacuum(False)
                #rospy.sleep(3)
                #pick(True)
                #rospy.sleep(1)
                #pick(False)
            rospy.sleep(1.0)
            step = time_count(step)

        # lettuce O 15

        elif step == 45:
            if "O" in ip_datas:
                pick(False)
                vacuum(False)
                command.look_shelf("middle","left")
                ShelfCommand("middle_open")
                ids = range(77, 82)
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    #if flag == False:
                        #vacuum(False)
                        #pick(True)
                        #command.sand_gripping_vertical(ids, True)
                        #flag = True
                        #pick(False)
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 48

        elif step == 46:
            if flag == False:
                command.look_shelf("middle","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    #if flag == False:
                        #pick(True)
                        #command.sand_gripping_vertical(ids,True)
                        #flag= True
                        #pick(False)
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)

        elif step == 47:
            if flag == True:
                command.bring_to_box()
                vacuum(False)
                #rospy.sleep(3)
                #pick(True)
                #rospy.sleep(1)
                #pick(False)
            rospy.sleep(1.0)
            step = time_count(step)


        # pouch P 16

        elif step == 48:
            if "P" in ip_datas:
                pick(False)
                vacuum(False)
                command.look_shelf("middle","left")
                ShelfCommand("middle_open")
                ids = range(82, 84)
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    if flag == False:
                        vacuum(False)
                        pick(True)
                        command.pouch_gripping_vertical(ids, True)
                        flag = True
                        pick(False)
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 51

        elif step == 49:
            if flag == False:
                command.look_shelf("middle","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    if flag == False:
                        pick(True)
                        command.pouch_gripping_vertical(ids, True)
                        flag= True
                        pick(False)
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)

        elif step == 50:
            if flag == True:
                command.bring_to_box()
                vacuum(False)
                rospy.sleep(3)
                pick(True)
                rospy.sleep(1)
                pick(False)
            rospy.sleep(1.0)
            step = time_count(step)


        # pouch Q 17
        elif step == 51:
            if "Q" in ip_datas:
                pick(False)
                vacuum(False)
                command.look_shelf("middle","left")
                ShelfCommand("middle_open")
                ids = range(84, 86)
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    if flag == False:
                        vacuum(False)
                        pick(True)
                        command.pouch_gripping_vertical(ids, True)
                        flag = True
                        pick(False)
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 54

        elif step == 52:
            if flag == False:
                command.look_shelf("middle","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    if flag == False:
                        pick(True)
                        command.pouch_gripping_vertical(ids, True)
                        flag= True
                        pick(False)
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)

        elif step == 53:
            if flag == True:
                command.bring_to_box()
                vacuum(False)
                rospy.sleep(3)
                pick(True)
                rospy.sleep(1)
                pick(False)
            rospy.sleep(1.0)
            step = time_count(step)

        # pouch R 18


        elif step == 54:
            if "R" in ip_datas:
                pick(False)
                vacuum(False)
                command.look_shelf("middle","left")
                ShelfCommand("middle_open")
                ids = range(86, 88)
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    if flag == False:
                        vacuum(False)
                        pick(True)
                        command.pouch_gripping_vertical(ids, True)
                        flag = True
                        pick(False)
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 57

        elif step == 55:
            if flag == False:
                command.look_shelf("middle","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.get_optimul_ar(ids)
                    vacuum(True)
                    if flag == False:
                        pick(True)
                        command.pouch_gripping_vertical(ids, True)
                        flag= True
                        pick(False)
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)

        elif step == 56:
            if flag == True:
                command.bring_to_box()
                vacuum(False)
                rospy.sleep(3)
                pick(True)
                rospy.sleep(1)
                pick(False)
            rospy.sleep(1.0)
            step = time_count(step)


##################################            
############### low ##############
##################################
        # coffee S 19
        elif step == 57:
            ShelfCommand("middle_close")
            spawner.remove_world()
            spawner.mikiwame_base()
            spawner.open_shelf("low") 
            if "S" in ip_datas:
                vacuum(False)
                command.look_shelf("low","left")
                ShelfCommand("low_open")
                rospy.sleep(5)
                ids = [88]
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.coffee_get_optimul_ar(ids)
                    vacuum(True)
                    command.coffee_pullup()
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)

            else:
                step = time_count(step)
                if step != 100:
                    step = 60

        elif step == 58:
            ShelfCommand("middle_close")
            if flag == False:
                command.look_shelf("low","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.coffee_get_optimul_ar(ids)
                    vacuum(True)
                    command.coffee_pullup()
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)            

        elif step == 59:
            if flag == True:
                command.coffee_bring_to_box()
                vacuum(False)
                rospy.sleep(3)
            rospy.sleep(1.0)
            step = time_count(step)

        # coffee T 20
        elif step == 60:
            if "T" in ip_datas:
                command.look_shelf("low","left")
                ShelfCommand("low_open")
                rospy.sleep(5)
                ids = [89]
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.coffee_get_optimul_ar(ids)
                    vacuum(True)
                    command.coffee_pullup()
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 63

        elif step == 61:
            if flag == False:
                command.look_shelf("low","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.coffee_get_optimul_ar(ids)
                    vacuum(True)
                    command.coffee_pullup()
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)            

        elif step == 62:
            if flag == True:
                command.coffee_bring_to_box()
                vacuum(False)
                rospy.sleep(3)
            rospy.sleep(1.0)
            step = time_count(step)


        # coffee U 21
        elif step == 63:
            if "U" in ip_datas:
                vacuum(False)
                command.look_shelf("low","left")
                ShelfCommand("low_open")
                rospy.sleep(5)
                ids = [90]
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.coffee_get_optimul_ar(ids)
                    vacuum(True)
                    command.coffee_pullup()
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 66

        elif step == 64:
            if flag == False:
                command.look_shelf("low","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.coffee_get_optimul_ar(ids)
                    vacuum(True)
                    command.coffee_pullup()
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)            

        elif step == 65:
            if flag == True:
                command.coffee_bring_to_box()
                vacuum(False)
                rospy.sleep(3)
            rospy.sleep(1.0)
            step = time_count(step)

    
        # katsu V 22
        elif step == 66:
            if "V" in ip_datas or "W" in ip_datas or  "X" in ip_datas:
                vacuum(False)
                command.look_shelf("low","left")
                ShelfCommand("low_open")
                rospy.sleep(5)
                ids = [91, 92, 93]
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.coffee_get_optimul_ar(ids)
                    vacuum(True)
                    command.coffee_pullup()
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 69

        elif step == 67:
            if flag == False:
                command.look_shelf("low","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.coffee_get_optimul_ar(ids)
                    vacuum(True)
                    command.coffee_pullup()
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)            

        elif step == 68:
            if flag == True:
                command.coffee_bring_to_box()
                vacuum(False)
                rospy.sleep(3)
            rospy.sleep(1.0)
            step = time_count(step)


        # katsu W 23
        elif step == 69:
            if "V" in ip_datas or "W" in ip_datas or  "X" in ip_datas:
                vacuum(False)
                command.look_shelf("low","left")
                ShelfCommand("low_open")
                rospy.sleep(5)
                ids = [91, 92, 93]
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.coffee_get_optimul_ar(ids)
                    vacuum(True)
                    command.coffee_pullup()
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 72

        elif step == 70:
            if flag == False:
                command.look_shelf("low","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.coffee_get_optimul_ar(ids)
                    vacuum(True)
                    command.coffee_pullup()
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)            

        elif step == 71:
            if flag == True:
                command.coffee_bring_to_box()
                vacuum(False)
                rospy.sleep(3)
            rospy.sleep(1.0)
            step = time_count(step)



        # katsu X 24
        elif step == 72:
            if "V" in ip_datas or "W" in ip_datas or  "X" in ip_datas:
                vacuum(False)
                command.look_shelf("low","left")
                ShelfCommand("low_open")
                rospy.sleep(5)
                ids = [91, 92, 93]
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.coffee_get_optimul_ar(ids)
                    vacuum(True)
                    command.coffee_pullup()
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 75

        elif step == 73:
            if flag == False:
                command.look_shelf("low","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.coffee_get_optimul_ar(ids)
                    vacuum(True)
                    command.coffee_pullup()
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)            

        elif step == 74:
            if flag == True:
                command.coffee_bring_to_box()
                vacuum(False)
                rospy.sleep(3)
            rospy.sleep(1.0)
            step = time_count(step)


        # nori Y 25
        elif step == 75:
            if "Y" in ip_datas or "Z" in ip_datas or "a" in ip_datas:
                vacuum(False)
                command.look_shelf("low","left")
                ShelfCommand("low_open")
                rospy.sleep(5)
                ids = [94, 95, 96]
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.coffee_get_optimul_ar(ids)
                    vacuum(True)
                    command.coffee_pullup()
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 78

        elif step == 76:
            if flag == False:
                command.look_shelf("low","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.coffee_get_optimul_ar(ids)
                    vacuum(True)
                    command.coffee_pullup()
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)            

        elif step == 77:
            if flag == True:
                command.coffee_bring_to_box()
                vacuum(False)
                rospy.sleep(3)
            rospy.sleep(1.0)
            step = time_count(step)

        # nori Z 26
        elif step == 78:
            if "Y" in ip_datas or "Z" in ip_datas or "a" in ip_datas:
                vacuum(False)
                command.look_shelf("low","left")
                ShelfCommand("low_open")
                rospy.sleep(5)
                ids = [94, 95, 96]
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.coffee_get_optimul_ar(ids)
                    vacuum(True)
                    command.coffee_pullup()
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 81

        elif step == 79:
            if flag == False:
                command.look_shelf("low","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.coffee_get_optimul_ar(ids)
                    vacuum(True)
                    command.coffee_pullup()
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)            

        elif step == 80:
            if flag == True:
                command.coffee_bring_to_box()
                vacuum(False)
                rospy.sleep(3)
            rospy.sleep(1.0)
            step = time_count(step)


        # nori a 27
        elif step == 81:
            if "Y" in ip_datas or "Z" in ip_datas or "a" in ip_datas:
                vacuum(False)
                command.look_shelf("low","left")
                ShelfCommand("low_open")
                rospy.sleep(5)
                ids = [94, 95, 96]
                flag = False
                left_ar_flag = command.exist_ar(ids)
                if left_ar_flag == True:
                    flag = command.coffee_get_optimul_ar(ids)
                    vacuum(True)
                    command.coffee_pullup()
                else:
                    pass
                rospy.sleep(1.0)
                step = time_count(step)
            else:
                step = time_count(step)
                if step != 100:
                    step = 84

        elif step == 82:
            if flag == False:
                command.look_shelf("low","right")
                right_ar_flag = command.exist_ar(ids)
                if right_ar_flag == True:
                    flag = command.coffee_get_optimul_ar(ids)
                    vacuum(True)
                    command.coffee_pullup()
                else:
                    pass
            rospy.sleep(1.0)
            step = time_count(step)            

        elif step == 83:
            if flag == True:
                command.coffee_bring_to_box()
                vacuum(False)
                rospy.sleep(3)
            rospy.sleep(1.0)
            step = time_count(step)
            #ShelfCommand("low_close")

        elif step == 84:
            ShelfCommand("low_close")
            move.y_move(-0.2)
            #step = 2
            step = time_count(step)
            if step != 100:
                step = 2
            retry_flag = True
            print("\n\n\n\n\n retry \n\n\n\n\n")
        
        """
        elif step == 84:
            ShelfCommand("low_close")
            move_goal()
            step = 9999
        """
##########################################################################
##########################################################################
############################ display #####################################
##########################################################################
##########################################################################
        """
        # onigiri ABC
        elif step == 84:
            ShelfCommand("low_close")
            #spawner.remove_world()
            #spawner.mikiwame_base()
            #spawner.open_shelf("low")
            #ShelfCommand("high_open")
            
            command.display("init", "ffffffff")
            pick(True)# open
            command.display("ABC", "pick")
            pick(False)
            rospy.sleep(1.0)
            step = time_count(step)

        elif step == 85:
            command.display("ABC", "place")
            ShelfCommand("high_open")
            pick(True)
            rospy.sleep(1.0)
            step = time_count(step)


        # onigiri DEF
        elif step == 86:
            ShelfCommand("high_close")          
            command.display("init", "ffffffff")
            pick(True)# open
            command.display("DEF", "pick")
            pick(False)
            rospy.sleep(1.0)
            step = time_count(step)

        elif step == 87:
            command.display("DEF", "place")
            ShelfCommand("high_open")
            pick(True)
            rospy.sleep(1.0)
            step = time_count(step)

        elif step == 88:
            step = 100
        """       
        if step == 100:
            if 1 <= save_step <= 29:
                vacuum(False)
                command.look_shelf("high", "left")
                ShelfCommand("high_close")
            elif 30 <= save_step <= 55:
                ShelfCommand("middle_close")
            elif 56 <= save_step <= 84:
                ShelfCommand("low_close")

            elif 85 <= save_step <= 87:
                ShelfCommand("high_close")

            #move_goal()
            step += 1
        


        r.sleep()

if __name__=='__main__':
    try:
        rospy.init_node('main',anonymous=True)
        command = XArm_command()
        spawner = Spawner()
        main()

    except rospy.ROSInterruptException:
        pass

