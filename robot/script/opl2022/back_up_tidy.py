#!/usr/bin/env python
# cooding: utf-8

import rospy
import smach
import smach_ros
import sys

sys.path.append('/home/demulab/opl_ws/src/OPL22/robot/script')
from module import *
import rosparam
import os
from std_msgs.msg import String, Float64, Bool
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
sys.path.append('/home/demulab/scout_ws/src/scout_ros/scout_navigation/srv')
from scout_navigation.srv import NaviLocation, NaviLocationResponse
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class DoorOpen(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['outcome1'])

    def execute(self,userdata):
        # voice.play("start_basic_functionalities.mp3")
        voice.play("please_open_the_door.mp3")
        arm.init_pose()
        move.navigation('entry') 
        return 'outcome1'

class PickAndPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['outcome1'])

    def pick(self):
        gripper.open()
        print(vision.target_object.pose.pose)
        if vision.target_object.pose.pose.position.x == 0 and vision.target_object.pose.pose.position.y == 0 and vision.target_object.pose.pose.position.z == 0:
            return
        target_pose_1 = geometry_msgs.msg.Pose()
        target_pose_1 = vision.target_object.pose.pose
        q = tf.transformations.quaternion_from_euler(0, math.pi / 3, math.pi)
        target_pose_1.position.x = vision.target_object.pose.pose.position.x - 0.15
        target_pose_1.position.z = vision.target_object.pose.pose.position.z + 0.07
        target_pose_1.orientation.x = q[0]
        target_pose_1.orientation.y = q[1]
        target_pose_1.orientation.z = q[2]
        target_pose_1.orientation.w = q[3]
        arm.move(target_pose_1) 

        rospy.sleep(1)
        target_pose_2 = geometry_msgs.msg.Pose()
        if vision.target_object.pose.pose.position.x == 0 and vision.target_object.pose.pose.position.y == 0 and vision.target_object.pose.pose.position.z == 0:
            return
        target_pose_2 = vision.target_object.pose.pose
        q = tf.transformations.quaternion_from_euler(0, math.pi / 8, math.pi)
        target_pose_2.position.x = vision.target_object.pose.pose.position.x + 0.03
        target_pose_2.orientation.x = q[0]
        target_pose_2.orientation.y = q[1]
        target_pose_2.orientation.z = q[2]
        target_pose_2.orientation.w = q[3]
        arm.move(target_pose_2) 
        gripper.close()

    def waste(self):
        target_pose_3 = geometry_msgs.msg.Pose()
        target_pose_3.position.x = 0.65
        target_pose_3.position.y = 0
        target_pose_3.position.z = 0.1
        q = tf.transformations.quaternion_from_euler(0, math.pi / 6, math.pi)
        target_pose_3.orientation.x = q[0]
        target_pose_3.orientation.y = q[1]
        target_pose_3.orientation.z = q[2]
        target_pose_3.orientation.w = q[3]
        arm.move(target_pose_3) 
 
    def execute(self,userdata):
        voice.play("start_pick_and_place.mp3")
        arm.init_pose()
        arm.add_table()
        move.navigation('tableB')
        rospy.sleep(1.0)
        self.pick()
        arm.init_pose()
        move.navigation('tableA')
        arm.waste()
        gripper.open()
        arm.init_pose()
        return 'outcome1'

class AvoidThat(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['outcome1'])
        self.pub_twist = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)
        self.twist_value = Twist()
        self.ac = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    def avoid(self, target_name): 
        location_dict = rosparam.get_param('/location')
        if target_name in location_dict:
             coord_list = location_dict[target_name]
             rospy.loginfo("Start Navigation")
 
             self.ac.wait_for_server()
             clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
 
             goal = MoveBaseGoal()
             goal.target_pose.header.frame_id = 'map'
             goal.target_pose.header.stamp = rospy.Time.now()
             goal.target_pose.pose.position.x = coord_list[0]
             goal.target_pose.pose.position.y = coord_list[1]
             goal.target_pose.pose.orientation.z = coord_list[2]
             goal.target_pose.pose.orientation.w = coord_list[3]
 
             clear_costmaps()
             rospy.wait_for_service('move_base/clear_costmaps')
             rospy.sleep(0.3)
             self.ac.send_goal(goal)
             state = self.ac.get_state()
             count = 0 # clear_costmap
             clear = 0
        while not rospy.is_shutdown():
                 state = self.ac.get_state()
                 if state == 1:
                     rospy.loginfo('Running...')
                     rospy.sleep(1.0)
                     clear += 1
                     if clear == 60:
                         return False
                 elif state == 3:
                     rospy.loginfo('Navigation success!!')
                     state = 0
                     return True
                 elif state == 4:
                      if count == 2:
                          count = 0
                          rospy.loginfo('Navigation Failed')
                          return False
                      else:
                          rospy.loginfo('Clearing Costmaps')
                          clear_costmaps()
                          self.ac.send_goal(goal)
                          rospy.loginfo('Send Goal')
                      count += 1

    def execute(self,userdata):
    """
        move.navigation('avoid_that')
        voice.play("start_avoid_that.mp3")
      #  self.avoid('dining')
        move.navigation('living')
      #  move.navigation('livingB')
        move.navigation('dining')
        move.navigation('what')"""
        return 'outcome1'

class WhatDidYouSay(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['outcome1'])

    def execute(self,userdata):
    """
        voice.play("start_what_did_you_say.mp3")"""
        return 'outcome1'

class Exit(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['outcome1'])

    def execute(self,userdata):
        #move.navigation('exit')
        return 'outcome1'

def main():
    move = Move()
    voice = Voice()
    sm_top = smach.StateMachine(outcomes = ['FINISH'])
    with sm_top:

        sm_do = smach.StateMachine(outcomes=['to_pap'])
        with sm_do:
            smach.StateMachine.add('DO', DoorOpen(),
                transitions={'outcome1':'to_pap'})
        smach.StateMachine.add('DOOR_OPEN',sm_do,
            transitions={'to_pap':'PICK_AND_PLACE'})

        sm_pap = smach.StateMachine(outcomes = ['to_AvoidThat'])
        with sm_pap:
            smach.StateMachine.add('PAP',PickAndPlace(),
                transitions={'outcome1':'to_AvoidThat'})
        smach.StateMachine.add('PICK_AND_PLACE',sm_pap,
            transitions={'to_AvoidThat':'AVOID_THAT'})


        sm_at = smach.StateMachine(outcomes = ['to_WhatDidYouSay'])
        with sm_at:
            smach.StateMachine.add('AT', AvoidThat(),
                transitions={'outcome1':'to_WhatDidYouSay'})
        smach.StateMachine.add('AVOID_THAT',sm_at,
            transitions={'to_WhatDidYouSay':'WHAT_DID_YOU_SAY'})
    
        sm_wdys = smach.StateMachine(outcomes = ['to_exit'])
        with sm_wdys:
            smach.StateMachine.add('WDYS', WhatDidYouSay(),
                transitions={'outcome1':'to_exit'})
        smach.StateMachine.add('WHAT_DID_YOU_SAY',sm_wdys,
            transitions={'to_exit':'EXIT'})

        sm_exit = smach.StateMachine(outcomes=['finish'])
        with sm_exit:
            smach.StateMachine.add('FINISH', Exit(),
                transitions={'outcome1':'finish'})
        smach.StateMachine.add('EXIT',sm_exit,
            transitions={'finish':'FINISH'})

#    outcome = sm_exit.execute()
    outcome = sm_top.execute()
            

if __name__ == '__main__':
    arm = Arm()
    gripper = Gripper()
    vision = Vision()
    voice = Voice()
    move = Move()


    try:
        rospy.init_node('basic_function', anonymous=True)
        main()
    except rospy.ROSInterruptException:
        pass
