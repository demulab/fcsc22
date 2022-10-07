#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import rosparam
from std_msgs.msg import String
from dynamixel_workbench_msgs.msg import DynamixelStateList
from dynamixel_workbench_msgs.srv import DynamixelCommand

class MotorController(object):
    def __init__(self):
        rospy.Subscriber('/dynamixel_workbench/dynamixel_state',DynamixelStateList,self.getMotorStateCB)
        self.motor_client = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command',DynamixelCommand)

        self.GRIPER_RANGE = 500
        self.calibration_flg = False
        self.current_pose = [2475]*1
        self.torque_error = [2475]*1
        self.rotation_velocity = [0]*1

    def calibration(self):
        self.CLOSE_POSITION = self.current_pose[0]
        self.OPEN_POSITION = self.CLOSE_POSITION + self.GRIPER_RANGE
        self.calibration_flg = True

    def getMotorStateCB(self, state):
        for i in range(1):
            self.current_pose[i] = state.dynamixel_state[i].present_position
            self.rotation_velocity[i] = abs(state.dynamixel_state[i].present_velocity)
            self.torque_error[i] = state.dynamixel_state[i].present_current
            print("pose", self.current_pose[i])

    def controlPosition(self, motor_id, rotate_value):
        if type(rotate_value) == type(float()):
            rotate_value = self.degToStep(rotate_value)
        res = self.motor_client('', motor_id, 'Goal_Position', rotate_value)

    def controlCurrent(self, motor_id, current_value):
        res = self.motor_client('', motor_id, 'Goal_Current', current_value)

    def degToStep(self,deg):
        return int((deg+180)/360.0*4095)

    def stepToDeg(self,step):
        return round(step/4095.0*360.0-180, 1)

class JointController(MotorController):
    def __init__(self):
        super(JointController, self).__init__()
        rospy.Subscriber('/servo/endeffector', String, self.controlEndeffector)
        #rospy.Subscriber('/servo/adsorption', Bool, self.controlAdsorption)

    def controlEndeffector(self, req):
        if not self.calibration_flg:
            self.calibration()
        if type(req) == type(String()):
            req = req.data
        if req == "OPEN":
            self.controlCurrent(4, 0)
            self.controlPosition(4, self.OPEN_POSITION)
            rospy.sleep(0.5)
            while self.rotation_velocity[0] > 0:
                pass
            else:
                self.controlCurrent(4, 500)
            rospy.loginfo("OPEN")
            return True
        elif req == "CLOSE":
            goal_position = self.CLOSE_POSITION
            grasp_flg = True
            self.controlCurrent(4, 500)
            self.controlPosition(4, goal_position)
            rospy.sleep(0.5)
        while self.rotation_velocity[0] > 0:
            pass
        else:
            self.controlPosition(4, self.current_pose[0])
            self.controlCurrent(4, 0)
        grasp_flg = self.torque_error[0] > 30 
        return grasp_flg

if __name__ == '__main__':
    rospy.init_node('motor_controller')
    experiment = JointController()
    rospy.spin()
