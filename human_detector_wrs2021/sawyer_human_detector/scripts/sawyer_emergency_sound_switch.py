#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sawyer_human_detector.msg import Emergency

def sawyer_sound_switch():
    rospy.init_node('sound_switch',anonymous=True)
    Sound = rospy.Publisher('human_detector', Emergency, queue_size=10)
    r = rospy.Rate(10)
    emergency_commander = Emergency()
    print("sawyer_sound_switch active")
    while not rospy.is_shutdown():
        key = raw_input()
        if key == 'a':
           emergency_commander.approch = 1
           emergency_commander.evacuation = 0
           emergency_commander.comeback = 0
           print("approch")
           print(emergency_commander.approch)
           print(emergency_commander.evacuation)
           print(emergency_commander.comeback)
        elif key == 'b':
           emergency_commander.approch = 0
           emergency_commander.evacuation = 1
           emergency_commander.comeback = 0
           print("evacuation")
           print(emergency_commander.approch)
           print(emergency_commander.evacuation)
           print(emergency_commander.comeback)
        elif key == 'c':
           emergency_commander.approch = 0
           emergency_commander.evacuation = 0
           emergency_commander.comeback = 1
           print("comeback")
           print(emergency_commander.approch)
           print(emergency_commander.evacuation)
           print(emergency_commander.comeback)
        else:
           print("input a, b, c")
           emergency_commander.approch = 0
           emergency_commander.evacuation = 0
           emergency_commander.comeback = 0
           print(emergency_commander.approch)
           print(emergency_commander.evacuation)
           print(emergency_commander.comeback)

        print(emergency_commander.approch)
        print(emergency_commander.evacuation)
        print(emergency_commander.comeback)
        Sound.publish(emergency_commander)
        r.sleep()

if __name__ == '__main__':
    try:
        sawyer_sound_switch()
    except rospy.ROSInterruptException:pass