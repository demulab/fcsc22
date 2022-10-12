#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import message_filters
from sawyer_human_detector.msg import Emergency
# from std_msgs.msg import Int32

class Robot():
    def __init__(self):
        self.detector_count = 0
        self.approach_count = 0
        self.restart = False
        self.restart_count = 0
        self.comeback = False
        self.comeback_count = 0
        self.step = 0
        self.sub1 = message_filters.Subscriber('human_detector_1', Emergency)
        self.sub2 = message_filters.Subscriber('human_detector_2', Emergency)

    def sawyer_human_detector_controll(self,msg1,msg2):
        Human_detector = rospy.Publisher('human_detector_command', Emergency, queue_size=1)
        emergency = Emergency()
        detector1 = msg1
        detector2 = msg2
        # print(self.step)
        self.step = -1
        if self.step < 0 or self.step > 7 and self.step < 35:
            print("step in")
            if self.detector_count > 20:
                if 0 < self.approach_count < 5:
                    if (detector1.humanlost == False and detector1.humandetectorcos <= 1.05 and detector1.humandetectorsin <= 1.65) or (detector2.humanlost == False and detector2.humandetectorsin <= 1.1 and detector2.humandetectorcos <= 1.85):
                        emergency.approach = True
                        emergency.sawyerstop = True
                        print("Human approaching. I need stop")
                        self.approach_count += 1
                        Human_detector.publish(emergency)
                    else:
                        emergency.approach = False
                        emergency.sawyerstop = False
                        emergency.restart = True
                        print("Human no approach. I restart work")
                        self.approach_count = 0
                        Human_detector.publish(emergency)
                        self.restart = True
                elif self.approach_count == 5:
                    if (detector1.humanlost == False and detector1.humandetectorcos <= 1.05 and detector1.humandetectorsin <= 1.65) or (detector2.humanlost == False and detector2.humandetectorsin <= 1.1 and detector2.humandetectorcos <= 1.85):
                        emergency.approach = False
                        emergency.evacuation = True
                        emergency.sawyerstop = True
                        print("Human approaching. I need evacuation")
                        self.approach_count += 1
                        Human_detector.publish(emergency)
                    else:
                        emergency.approach = False
                        emergency.sawyerstop = False
                        emergency.restart = True
                        print("Human no approach. I restart work") 
                        self.approach_count = 0
                        Human_detector.publish(emergency)
                        self.restart = True
                elif 5 < self.approach_count:
                    if (detector1.humanlost == False and detector1.humandetectorcos <= 1.05 and detector1.humandetectorsin <= 1.65) or (detector2.humanlost == False and detector2.humandetectorsin <= 1.1 and detector2.humandetectorcos <= 1.85):
                        emergency.evacuation = True
                        emergency.sawyerstop = True
                        print("Human approaching. I need evacuation")
                        self.approach_count += 1
                        Human_detector.publish(emergency)
                    else:
                        emergency.evacuation = False
                        emergency.sawyerstop = False
                        emergency.comeback = True
                        print("Human no approach. I comeback work")
                        self.approach_count = 0
                        Human_detector.publish(emergency)
                        self.comeback = True
                else:
                    if (detector1.humanlost == False and detector1.humandetectorcos <= 1.05 and detector1.humandetectorsin <= 1.65) or (detector2.humanlost == False and detector2.humandetectorsin <= 1.1 and detector2.humandetectorcos <= 1.85):
                        emergency.approach = True
                        print("Human approach")
                        self.approach_count += 1
                        Human_detector.publish(emergency)
                    elif self.restart == True:
                        if self.restart_count > 3:
                            self.restart = False
                            self.restart_count = 0
                            self.comeback = False
                            self.comeback_count = 0
                            emergency.approach = False
                            emergency.sawyerstop = False
                            emergency.restart = False
                            emergency.comeback = False
                            print("Human no approach")
                            self.approach_count = 0
                            Human_detector.publish(emergency)
                        else:
                            emergency.approach = False
                            emergency.sawyerstop = False
                            emergency.restart = True
                            emergency.comeback = False
                            print("Human no approach. I restart work") 
                            Human_detector.publish(emergency)
                            self.restart_count += 1
                    elif self.comeback == True:
                        if self.comeback_count > 3:
                            self.restart = False
                            self.restart_count = 0
                            self.comeback = False
                            self.comeback_count = 0
                            emergency.approach = False
                            emergency.sawyerstop = False
                            emergency.restart = False
                            emergency.comeback = False
                            print("Human no approach")
                            self.approach_count = 0
                            Human_detector.publish(emergency)
                        else:
                            emergency.approach = False
                            emergency.sawyerstop = False
                            emergency.restart = False
                            emergency.comeback = True
                            print("Human no approach. I comeback work")
                            Human_detector.publish(emergency)
                            self.comeback_count += 1
                    else:
                        emergency.approach = False
                        emergency.sawyerstop = False
                        emergency.restart = False
                        emergency.comeback = False
                        print("Human no approach")
                        self.approach_count = 0
                        Human_detector.publish(emergency)
                self.detector_count = 0
            else:
                self.detector_count += 1

    def robotMecanumMoveJudge(self,msg):
        self.step = msg.mecanum

        
if __name__ == "__main__":
    rospy.init_node("human_emergency_controll", anonymous=True)
    print("sawyer_human_detector_controll active")
    robot = Robot()
    rospy.Subscriber("/sawyer_to_lidar", Emergency, robot.robotMecanumMoveJudge)
    fps = 33.0
    delay = 1/fps
    ts = message_filters.ApproximateTimeSynchronizer([robot.sub1,robot.sub2], 1, delay)
    ts.registerCallback(robot.sawyer_human_detector_controll)

    rospy.spin()
        
