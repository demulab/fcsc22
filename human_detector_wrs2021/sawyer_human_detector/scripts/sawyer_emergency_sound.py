#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pyaudio  
import wave
from sawyer_human_detector.msg import Emergency

chunk = 1024

class Robot():
    def __init__(self):
        self.approach_sound = False
        self.evacuation_sound = False
        self.restart_comeback_sound = False
    def sawyer_sound(self, msg):
        emergency = msg
        if emergency.approach == True and self.approach_sound == False and self.restart_comeback_sound == False:
            print("approach")
            f = wave.open("/home/demulab/opl_ws/src/sawyer_wrs2021/sawyer_human_detector_wrs2021/sawyer_human_detector/wav/approach.wav","rb")
            p = pyaudio.PyAudio()
            stream = p.open(format=p.get_format_from_width(f.getsampwidth()), channels=f.getnchannels(), rate=f.getframerate(), output=True)
            data = f.readframes(chunk)
            while data != '':
                stream.write(data)
                data = f.readframes(chunk)
            stream.stop_stream()
            stream.close()
            p.terminate()
            self.approach_sound = True
            self.restart_comeback_sound = True
        elif emergency.evacuation == True and self.evacuation_sound == False:
            print("evacuation")
            f = wave.open("/home/demulab/opl_ws/src/sawyer_wrs2021/sawyer_human_detector_wrs2021/sawyer_human_detector/wav/evacuation.wav","rb")
            p = pyaudio.PyAudio()
            stream = p.open(format=p.get_format_from_width(f.getsampwidth()), channels=f.getnchannels(), rate=f.getframerate(), output=True)
            data = f.readframes(chunk)
            while data != '':
                stream.write(data)
                data = f.readframes(chunk)
            stream.stop_stream()
            stream.close()
            p.terminate()
            self.evacuation_sound = True
            self.restart_comeback_sound = True
        elif (emergency.comeback == True or emergency.restart == True) and self.restart_comeback_sound == True:
            print("comeback or restart")
            f = wave.open("/home/demulab/sawyer_ws/src/sawyer_wrs2021/sawyer_human_detector_wrs2021/sawyer_human_detector/wav/comeback.wav","rb")
            p = pyaudio.PyAudio()
            stream = p.open(format=p.get_format_from_width(f.getsampwidth()), channels=f.getnchannels(), rate=f.getframerate(), output=True)
            data = f.readframes(chunk)
            while data != '':
                stream.write(data)
                data = f.readframes(chunk)
            stream.stop_stream()
            stream.close()
            p.terminate()
            self.approach_sound = False
            self.evacuation_sound = False
            self.restart_comeback_sound = False
        elif emergency.approach == False and emergency.evacuation == False and emergency.restart == False and emergency.comeback == False and self.restart_comeback_sound == True and (self.evacuation_sound == True or self.approach_sound == True):
            print("comeback")
            f = wave.open("/home/ren/opl_ws/src/sawyer_wrs2021/sawyer_human_detector_wrs2021/sawyer_human_detector/wav/comeback.wav","rb")
            p = pyaudio.PyAudio()
            stream = p.open(format=p.get_format_from_width(f.getsampwidth()), channels=f.getnchannels(), rate=f.getframerate(), output=True)
            data = f.readframes(chunk)
            while data != '':
                stream.write(data)
                data = f.readframes(chunk)
            stream.stop_stream()
            stream.close()
            p.terminate()
            self.approach_sound = False
            self.evacuation_sound = False
            self.restart_comeback_sound = False
    
rospy.init_node("emergency_sound", anonymous=True)

robot = Robot()
cxy_sub = rospy.Subscriber('human_detector_command', Emergency, robot.sawyer_sound, queue_size=1)

print("sawyer_sound_controller activate")

rospy.spin()
