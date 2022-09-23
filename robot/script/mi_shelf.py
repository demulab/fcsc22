#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
#import intera_dataflow
#from intera_io import IODeviceInterface
#from intera_core_msgs.msg import IONodeConfiguration
import rospkg
#import intera_interface
from gazebo_msgs.srv import (
  SpawnModel,
  DeleteModel,
)
from geometry_msgs.msg import (
  PoseStamped,
  Pose,
  Point,
  Quaternion,
)
# from geometry_msgs.msg import Pose, PoseStamped
# from moveit_msgs.msg import Constraints, OrientationConstraint

class Manipulation(object):
  def __init__(self):
    print("***Manipulation_initial***")
    super(Manipulation, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface() 
    self.group = moveit_commander.MoveGroupCommander("right_arm")
    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
    self.target_pose = geometry_msgs.msg.Pose()
    self.target_pose_sub = rospy.Subscriber("/ar_pose", Pose, self.target_pose_callback)

  def plan(self):
    print("***generate_plan***") 
    self.group.set_pose_target(self.target_pose)
    plan = self.group.plan()    
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    self.display_trajectory_publisher.publish(display_trajectory);
    rospy.sleep(3)
  
  def execute(self):
    print("***execute***")
    self.group.go(wait=True)
    self.group.clear_pose_targets()
  
  def plan_and_execute(self):
    self.plan()
    self.execute()

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
    q = tf.transformations.quaternion_from_euler(roll * 3.14 / 180, pitch * 3.14 / 180, yaw * 3.14 / 180)
    self.target_pose.position.x = x
    self.target_pose.position.y = y
    self.target_pose.position.z = z
    self.target_pose.orientation.x = q[0]
    self.target_pose.orientation.y = q[1]
    self.target_pose.orientation.z = q[2]
    self.target_pose.orientation.w = q[3]
  
  def get_current_state(self):
    print("***get_current_state***")
    return self.robot.get_current_state()

  def generate_obstacle_box(self):
    print("***generate_obstacle_box***")

  def remove_obstacle_box(self):
    print("***remove_obstacle_box***")

  def target_pose_callback(self, msg):
    print("***target_pose_callback***")
    print("(x,y,z) = (%3.3f: %3.3f: %3.3f), (qx,qy,qz, qz) = (%3.3f: %3.3f: %3.3f: %3.3f)"% (msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))


class ShioharaEndEffector(object): 
  def __init__(self):
    print("***ShioharaEndEffector_initial***")
    super(ShioharaEndEffector, self).__init__()

  def suction_on(self):
    print("***suction_on***")

  def suction_off(self):
    print("***suction_off***")

  def suction_stretch(self):
    print("***suctio_stretch***")

  def suction_shrink(self):
    print("***suction_shrink***")
  
  def griper_open(self):
    print("***gripper_open***")
  
  def griper_close(self):
    print("***gripper_close***")
   
    
class Sawyer(Manipulation, ShioharaEndEffector):
  def __init__(self):
    print("***Sawyer_initial***")
    super(Sawyer, self).__init__()
    # self.limb = intera_interface.limb('right')

  def initial_state(self):
    print("***initial_state***") 
    initial_angles = {'right_l6': 0.0, 'right_l5': 0.0,'right_l4': 0.0,'right_l3': 0.0,'right_l2': 0.0,'right_l1': 0.0,'right_l0': 0.0}
    self.limb.move_to_joint_positions(initial_angles)

  def neutral(self):
    print("***neutral***")
    self.limb.move_to_neutral()

  def joint_revolution(self, joint_name, angle):
    print("***joint_revolution***")
    current_joints = self.limb.joint_angles()
    current_joints[joint_name] = angle
    self.limb.move_to_joint_positions(current_joints)

  def lights(self):
    print("***lights***")

  def get_current_tip_pose(self):
    print("***get_current_tip_pose***")
    tip_listener = tf.TransformListener()
    tip_listener.waitForTransform("/stp_021709TP00128_tip", "/base", rospy.Time(0), rospy.Duration(1.0))
    (xyz, q) = tip_listener.lookupTransform("/base", "/stp_021709TP00128_tip", rospy.Time(0))
    rpy = tf.transformations.euler_from_quaternion((q[0], q[1], q[2], q[3]))
    print(rpy[0] * 180 / 3.14)
    print(rpy[1] * 180 / 3.14)
    print(rpy[2] * 180 / 3.14)
    return xyz, rpy 

  def shelf_base_mesh(self, x, y, z, qx, qy, qz, qw):# shelf_base_add
    # Add a mesh to the planning scene
    rospy.sleep(2)  # Waiting for PlanningSceneInterface
    s_pose = PoseStamped()
    s_pose.header.frame_id = self.group.get_planning_frame()
    s_pose.pose.position.x = x
    s_pose.pose.position.y = y
    s_pose.pose.position.z = z
    s_pose.pose.orientation.x = qx
    s_pose.pose.orientation.y = qy
    s_pose.pose.orientation.z = qz
    s_pose.pose.orientation.w = qw
    self.scene.add_mesh("shelf_base", s_pose, "base_link.stl",(0.001,0.001,0.001))

  def shelf_board_mesh(self, x, y, z, qx, qy, qz, qw):# shelf_board_add
    # Add a mesh to the planning scene
    rospy.sleep(2)  # Waiting for PlanningSceneInterface
    s_pose = PoseStamped()
    s_pose.header.frame_id = self.group.get_planning_frame()
    s_pose.pose.position.x = x
    s_pose.pose.position.y = y
    s_pose.pose.position.z = z
    s_pose.pose.orientation.x = qx
    s_pose.pose.orientation.y = qy
    s_pose.pose.orientation.z = qz
    s_pose.pose.orientation.w = qw
    self.scene.add_mesh("shelf_board_1", s_pose, "Link1_1.stl",(0.001,0.001,0.001))

  def car_mesh(self, x, y, z, qx, qy, qz, qw):# shelf_board_add
    # Add a mesh to the planning scene
    rospy.sleep(2)  # Waiting for PlanningSceneInterface
    s_pose = PoseStamped()
    s_pose.header.frame_id = self.group.get_planning_frame()
    s_pose.pose.position.x = 0
    s_pose.pose.position.y = 0.25
    s_pose.pose.position.z = 0
    s_pose.pose.orientation.x = 0
    s_pose.pose.orientation.y = 0
    s_pose.pose.orientation.z = 0
    s_pose.pose.orientation.w = 1
    self.scene.add_mesh("car_1", s_pose, "car.stl",(0.001,0.001,0.001))

  def remove_object(self, object):
    rospy.sleep(2)
    self.scene.remove_world_object(object)

  def remove_world(self):
    rospy.sleep(2)
    self.scene.remove_world_object()

  """def init_upright_path_constraints(self):
    self.upright_constraints = Constraints()
    self.upright_constraints.name = "down"
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header = self.group.get_planning_frame()
    orientation_constraint.link_name = self.get_end_effector_link()
    orientation_constraint.orientation = target_pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = 3.1415
    orientation_constraint.absolute_y_axis_tolerance = 0.05
    orientation_constraint.absolute_z_axis_tolerance = 0.05
    #orientation_constraint.absolute_z_axis_tolerance = 3.14 #ignore this axis
    orientation_constraint.weight = 1.0
    self.upright_constraints.orientation_constraints.append(orientation_constraint)
    self.arm.set_path_constraints(self.upright_constraints)
  # def enable_upright_path_constraints(self):
  #   self.arm.set_path_constraints(self.upright_constraints)"""
    
def main():
  print("start manipulation.py")
  rospy.init_node("sawyer_moveit", anonymous=True)
   
  sawyer = Sawyer()
  # rospy.spin()
  # while not rospy.is_shutdown():
    # print(sawyer.get_current_tip_pose())
  

  # sawyer.set_target_pose_quaternion(0.451, 0.723, 0.161, -0.026, -0.031, -0.729, -0.683)
  # sawyer.init_upright_path_constraints()
  # sawyer.set_target_pose_quaternion(0.451, 0.723, 0.161, -0.026, -0.031, -0.729, -0.683)
  # sawyer.set_target_pose_euler(0.722, 0.443, 0.311, -1.0 * -3, 37 + 180, 179)
  #障害物
  sawyer.car_mesh(0, 0.25, 0, 0, 0, 0, 1)
  sawyer.shelf_base_mesh(1, 0, -0.9, 0, 0, -1, 1)
  sawyer.shelf_board_mesh(1, 0, -0.9, 0, 0, -1, 1)
  sawyer.shelf_board_mesh(0.7, 0, -0.9, 0, 0, -1, 1)#shelf_open
  rospy.sleep(2)
  sawyer.shelf_board_mesh(1, 0, -0.9, 0, 0, -1, 1)#shelf_close
  rospy.sleep(2)
  sawyer.remove_object("car_1")
  sawyer.remove_world()



 # sawyer.plan()
 # sawyer.execute()
  
if __name__=='__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
