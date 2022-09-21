#!/usr/bin/env python
# -*- coding: utf-8 -*-
#import rospy
import subprocess

from Ir.msg import Ir

def irShelfController(msg):
    shelf_command = msg.irsignal

    if shelf_command == "high_shelf_open":
        for i in range(0,2):
            res = subprocess.call(["python","/home/catkin_ws/src/maximum/src/auto_shelif/irmcli/irmcli.py","-p","-f","/home/catkin_ws/src/maximum/src/auto_shelif/json/open_high_shelf.json"])
        if res == 0:
            print("high_shelf_open")
    elif shelf_command == "high_shelf_close":
        for i in range(0,2):
            res = subprocess.call(["python","/home/catkin_ws/src/maximum/src/auto_shelif/irmcli/irmcli.py","-p","-f","/home/catkin_ws/src/maximum/src/auto_shelif/json/close_high_shelf.json"])
        if res == 0:
            print("high_shelf_close")
    elif shelf_command == "high_shelf_display":
        for i in range(0,2):
            res = subprocess.call(["python","/home/catkin_ws/src/maximum/src/auto_shelif/irmcli/irmcli.py","-p","-f","/home/catkin_ws/src/maximum/src/auto_shelif/json//close_middle_shelf.json"])
        if res == 0:
            print("high_shelf_display")
    elif shelf_command == "middle_shelf_open":
        for i in range(0,2):
            res = subprocess.call(["python","/home/catkin_ws/src/maximum/src/auto_shelif/irmcli/irmcli.py","-p","-f","/home/catkin_ws/src/maximum/src/auto_shelif/json/open_middle_shelf.json"])
        if res == 0:
            print("middle_shelf_open")
    elif shelf_command == "middle_shelf_close":
        for i in range(0,2):
            res = subprocess.call(["python","/home/catkin_ws/src/maximum/src/auto_shelif/irmcli/irmcli.py","-p","-f","/home/catkin_ws/src/maximum/src/auto_shelif/json/close_middle_shelf.json"])
        if res == 0:
            print("middle_shelf_close")
    elif shelf_command == "middle_shelf_display":
        for i in range(0,2):
            res = subprocess.call(["python","/home/catkin_ws/src/maximum/src/auto_shelif/irmcli/irmcli.py","-p","-f","/home/catkin_ws/src/maximum/src/auto_shelif/json/close_low_shelf.json"])
        if res == 0:
            print("middle_shelf_display")
    elif shelf_command == "low_shelf_open":
        for i in range(0,2):
            res = subprocess.call(["python","/home/catkin_ws/src/maximum/src/auto_shelif/irmcli/irmcli.py","-p","-f","/home/catkin_ws/src/maximum/src/auto_shelif/json/open_low_shelf.json"])
        if res == 0:
            print("low_shelf_open")
    elif shelf_command == "low_shelf_close":
        for i in range(0,2):
            res = subprocess.call(["python","/home/catkin_ws/src/maximum/src/auto_shelif/irmcli/irmcli.py","-p","-f","/home/catkin_ws/src/maximum/src/auto_shelif/json/close_low_shelf.json"])
        if res == 0:
            print("low_shelf_close")
    elif shelf_command == "low_shelf_display":
        for i in range(0,2):
            res = subprocess.call(["python","/home/catkin_ws/src/maximum/src/auto_shelif/irmcli/irmcli.py","-p","-f","/home/catkin_ws/src/maximum/src/auto_shelif/json/close_high_shelf.json"])
        if res == 0:
            print("low_shelf_display")

rospy.init_node('irmagician_active', anonymous=True)

irmagician_sub = rospy.Subscriber('ir_shelf_controller', Ir, irShelfController, queue_size=1)

print("irmagician_acitive activate")

rospy.spin()
	
