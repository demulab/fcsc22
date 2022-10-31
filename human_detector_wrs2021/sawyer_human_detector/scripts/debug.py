#!/usr/bin/env python
# cooding: utf-8

import rospy 
from std_msgs.msg import Bool, Int32
from sawyer_human_detector.msg import Emergency

def humanDetectCB(msg):
    if msg.sawyerstop == True:
        print("sub_robot_stop")

def main():
    rospy.Subscriber("human_detector_command", Emergency, humanDetectCB)
    print("aaa")
    while(1):
        rospy.sleep(1.0)

if __name__=='__main__':
    rospy.init_node("debug", anonymous=True)
    main()
