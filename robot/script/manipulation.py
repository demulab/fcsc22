#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
import tf
import rospkg
from geometry_msgs.msg import (
  PoseStamped,
  Pose,
  Point,
  Quaternion,
)
import actionlib
from xarm.msg import MoveAction
# from xarm_gripper.msg import GripperAction

class Manipulation(object):
    def __init__(self):
        print("***Manipulation_initial***")
        super(Manipulation, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface() 
        self.group = moveit_commander.MoveGroupCommander("xarm6") 
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory) 
        self.target_pose = geometry_msgs.msg.Pose()
        self.ar_pose = geometry_msgs.msg.Pose()
        rospy.Subscriber("/ar_pose", Pose, self.target_pose_callback)
        #added wrs2022
        self.group.set_planning_time(7)

    def plan(self):
        print("***generate_plan***") 
        self.group.set_max_velocity_scaling_factor(1.0)
        self.group.set_pose_target(self.target_pose)
        plan = self.group.plan()    
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        rospy.sleep(1)
        return plan 
 
    def waste(self):
        joint_list_2 = [0.06951850652694702, 0.667941153049469, -1.3437280654907227, 0.007408507633954287, 0.7364147305488586, 0.0637318342924118]
        self.group.set_joint_value_target("joint1", joint_list_2[0])
        self.group.set_joint_value_target("joint2", joint_list_2[1])
        self.group.set_joint_value_target("joint3", joint_list_2[2])
        self.group.set_joint_value_target("joint4", joint_list_2[3])
        self.group.set_joint_value_target("joint5", joint_list_2[4])
        self.group.set_joint_value_target("joint6", joint_list_2[5])
        self.group.go()
  
    def straight_plan(self, division_num, start_pose, distance):
        waypoints = []
        point_pose = Pose()
        for i in range(1, division_num + 1):
            point_pose.position.x = start_pose.position.x - (distance / division_num) * i 
            point_pose.position.y = start_pose.position.y
            point_pose.position.z = start_pose.position.z
            point_pose.orientation.x = start_pose.orientation.x
            point_pose.orientation.y = start_pose.orientation.y
            point_pose.orientation.z = start_pose.orientation.z
            point_pose.orientation.w = start_pose.orientation.w 
            waypoints.append(copy.deepcopy(point_pose))
            print(waypoints)
            (plan,fraction) = self.group.compute_cartesian_path(waypoints,0.01,0.0)
            print(plan)
            self.group.execute(plan)
  
    def execute(self):
        print("***execute***")
        self.group.go(wait=True)
        self.group.clear_pose_targets()
  
    def plan_and_execute(self):
        self.plan()
        self.execute()

    def stop(self):
        self.group.stop() 

    def set_target_pose_quaternion(self, x, y, z, qx, qy, qz, qw):
        print("***set_pose_quaternion***")
        self.target_pose.position.x = x
        self.target_pose.position.y = y
        self.target_pose.position.z = z
        self.target_pose.orientation.x = qx
        self.target_pose.orientation.y = qy
        self.target_pose.orientation.z = qz
        self.target_pose.orientation.w = qw
 
    def set_target_pose_euler(self, x, y, z, roll, pitch, yaw):
        print("***set_pose_euler***")
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        self.target_pose.position.x = x
        self.target_pose.position.y = y
        self.target_pose.position.z = z
        self.target_pose.orientation.x = q[0]
        self.target_pose.orientation.y = q[1]
        self.target_pose.orientation.z = q[2]
        self.target_pose.orientation.w = q[3]

    def target_pose_callback(self, msg):
        # print("***target_pose_callback***")
        self.ar_pose.position.x = msg.position.x
        self.ar_pose.position.y = msg.position.y
        self.ar_pose.position.z = msg.position.z
        self.ar_pose.orientation.x = msg.orientation.x
        self.ar_pose.orientation.y = msg.orientation.y
        self.ar_pose.orientation.z = msg.orientation.z
        self.ar_pose.orientation.w = msg.orientation.w
   
    def addTable(self):
        name = "Table"
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = 0.4 + 1.0 / 2
        p.pose.position.y = 0.
        p.pose.position.z = 0.0 - 0.4 / 2
        self.scene.add_box(name, p, (1.0, 1.5, 0.4))

class xArm6(Manipulation):
    def __init__(self):
        print("***xArm6_initial***")
        super(xArm6, self).__init__()
        self._xarm_server = actionlib.SimpleActionServer('xarm6_moveit', MoveAction, execute_cb = self.move, auto_start = False)
        self._xarm_server.register_preempt_callback(self.stop)
        self._xarm_server.start()
    
    def move(self, goal):
        print("move")
        feedback = MoveAction().action_feedback.feedback
        result = MoveAction().action_result.result
        if(goal.mode == True):
            print("joint")
            joint_list = [-0.1770578771829605, -1.3292371034622192, -0.5946793556213379, 0.2200702428817749, 0.9829257726669312, -0.23787248134613037]
            self.group.set_joint_value_target("joint1", joint_list[0])
            self.group.set_joint_value_target("joint2", joint_list[1])
            self.group.set_joint_value_target("joint3", joint_list[2])
            self.group.set_joint_value_target("joint4", joint_list[3])
            self.group.set_joint_value_target("joint5", joint_list[4])
            self.group.set_joint_value_target("joint6", joint_list[5])
            self.group.go()
        elif(goal.mode == False):
            print("point")
            rpy = tf.transformations.euler_from_quaternion((goal.target_pose.orientation.x, goal.target_pose.orientation.y, goal.target_pose.orientation.z, goal.target_pose.orientation.w))
            self.set_target_pose_euler(goal.target_pose.position.x, goal.target_pose.position.y + 0.05, goal.target_pose.position.z + 0.04, -1 * rpy[0], math.pi + rpy[1], rpy[2])
            plan = self.plan()
            #print("plan", plan)
            if not plan.joint_trajectory.points:
                feedback.plan = False
                self._xarm_server.publish_feedback(feedback)
                result.success = False
            else:
                feedback.plan = True
                self._xarm_server.publish_feedback(feedback)
                self.execute() 
                result.success = True
            self._xarm_server.set_succeeded(result = result)

    

    def stop(self):
        print("stop")
        self.stop()


if __name__=='__main__':
    try:
        rospy.init_node("xArm6_Moveit", anonymous = True)
        xArm6()
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass
