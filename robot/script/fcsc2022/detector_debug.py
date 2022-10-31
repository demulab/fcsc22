#!/usr/bin/env python
# cooding: utf-8

import sys
#sys.path.append('/home/demulab/opl_ws/src/fcsc22/robot/script')
#sys.path.append('/home/demulab/opl_ws/src/fcsc22/human_detector_wrs2021/sawyer_human_detector')
import tf
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
sys.path.append('/home/demulab/opl_ws/src/fcsc22/robot/script')
from module import *
from manipulation import *
from fcsc_command_v3 import *
from Moveit_mikiwame import *
import rosparam
import os
sys.path.append('/home/demulab/opl_ws/src/fcsc22/human_detector_wrs2021/sawyer_human_detector')
from sawyer_human_detector.msg import Emergency

#state
step = 0
save_step = 0
save_step_flg = False
hoge = False


def humanDetectCB(msg):
    global save_step
    global save_step_flg
    global step
    global hoge
    print("ooooooo")
    print(msg.sawyerstop)
    if msg.sawyerstop ==True:
        print("sub_sawyer_stop")
        if save_step_flg == False:
            save_step = step
            save_step_flg = True
            print("save_step",save_step)
            step = -2
            print("step",step)


def main():
    #rospy.init_node('main',anonymous=True)
    emergency = Emergency()
    #pub
    emergency_pub = rospy.Publisher('sawyer_to_lidar', Emergency, queue_size=10)

    #sub
    rospy.Subscriber("human_detector_command", Emergency,humanDetectCB)

   # global step
   # global hoge
   # global save_step
    r = rospy.Rate(10)
    while not rospy.is_shutdown() or  not step == 29:
        global step
        print('---step=%s--' %(step))

        emergency.mecanum = step
        emergency_pub.publish(emergency)

        if step == 0:
            #spawner.remove_world()
            #command.look_shelf("high","left")

#################################
            #move_shelf()
################################
#            get_id()
            rospy.sleep(2.0)
            step += 1

        elif step == -1:
            print("stop")
            global hoge
            global save_step
           # print(hoge)
            if hoge == False:
                print("save_step",save_step)

                if 1 <= save_step <= 29:
                    print("haigh_shelf_close")
                elif 30 <= save_step <= 55:
                    print("middle_shelf_close")

                    #irmachisin
                elif 56 <= save_step <= 100:
                    print("low_shelf_close")
                   #irmagician
                hoge = True
                step = save_step

                rospy.sleep(5.0)
                print("step",step)
                if 1 <= step <= 29:
                    print("high_shelf_open")
                    #irmacgisyan
                elif 30 <= step <= 55:
                    print("middle_shelf_opne")
                #irmachisin
                elif 56 <= step <= 100:
                    print("low_shelf_open")
             #irmagician

        elif step == -2:
            print("stop")
            global hoge
            global save_step
            print("hoge",hoge)
            print("save_step",save_step)
            if hoge == False:
                print("hoge_false")
                mani.stop()
                if 1 <= save_step <= 29:
                    print("haigh_shelf_close")
                     #irmacgisyanclose
                elif 30 <= save_step <= 55:
                    print("middle_shelf_close")

                     #irmachisin
                elif 56 <= save_step <= 100:
                    print("high_shelf_close")
                hoge = True
                step = save_step

                rospy.sleep(5.0)
                #move.x_move(-1.25)
                print("step",step)
                if 1 <= step <= 29:
                    print("high_shelf_open")
                     #irmacgisyan
                elif 30 <= step <= 55:
                    print("middle_shelf_opne")
                 #irmachisin
                elif 56 <= step <= 100:
                    print("low_shelf_open")


        elif step == 1:
            rospy.sleep(1.0)
            step += 1
##########################################################
##################### high ###############################
##########################################################
        # call planning scene
        elif step == 2:
            rospy.sleep(1.0)
            step += 1
            # onigiri A 01

        elif step == 3:
            rospy.sleep(1.0)
            step += 1

        elif step == 4:
            rospy.sleep(1.0)
            step += 1

        # onigiri B 02
        elif step == 5:
            rospy.sleep(1.0)
            step += 1

        elif step == 6:
            rospy.sleep(1.0)
            step = 1


        r.sleep()

if __name__=='__main__':
    try:
        rospy.init_node('main',anonymous=True)
       # command = XArm_command()
       # spawner = Spawner()
        main()

    except rospy.ROSInterruptException:
        pass
