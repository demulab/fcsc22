#!/usr/bin/env python
# cooding: utf-8

import sys
import rospy
sys.path.append('/home/demulab/opl_ws/src/OPL22/robot/script')
from module import Voice

def main():
    voice = Voice()
    voice.text_to_mp3("please open the door", "please_open_the_door.mp3")
    voice.play("please_open_the_door.mp3")
    voice.text_to_mp3("success inspection", "success_inspection.mp3")
    voice.play("success_inspection.mp3")
    voice.text_to_mp3("start pick and place", "start_pick_and_place.mp3")
    voice.play("start_pick_and_place.mp3")
    voice.text_to_mp3("start avoid that", "start_avoid_that.mp3")
    voice.play("start_avoid_that.mp3")
    voice.text_to_mp3("start what did you say", "start_what_did_you_say.mp3")
    voice.play("start_what_did_you_say.mp3")
    voice.text_to_mp3("success basic functionalities", "success_basic_functionalities.mp3")
    voice.play("success_basic_functionalities.mp3")
    voice.text_to_mp3("start inspection", "start_inspection.mp3")
    voice.play("start_inspection.mp3")
    voice.text_to_mp3("start basic functionalities", "start_basic_functionalities.mp3")
    voice.play("start_basic_functionalities.mp3")
    voice.text_to_mp3("start tidyup", "start_tidyup.mp3")
    voice.play("start_tidyup.mp3")
    voice.text_to_mp3("start task2A", "start_task2A.mp3")
    voice.play("start_task2A.mp3")
    voice.text_to_mp3("start task2B", "start_task2B.mp3")
    voice.play("start_task2B.mp3")
    voice.text_to_mp3("Those on the right, please extend your hand.", "Those_on_the_right_please_extend_your_hand.mp3")
    voice.play("Those_on_the_right_please_extend_your_hand.mp3")
    voice.text_to_mp3("Those on the left, please extend your hand.", "Those_on_the_left_please_extend_your_hand.mp3")
    voice.play("Those_on_the_left_please_extend_your_hand.mp3")
    voice.text_to_mp3("start task1", "start_task1.mp3")
    voice.play("start_task1.mp3")





if __name__ == '__main__':
    try:
        main() 
    except rospy.ROSInterruptException:
        pass
