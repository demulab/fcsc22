#!/usr/bin/env python

import rospy
import tf
import geometry_msgs.msg
from geometry_msgs.msg import (
  PoseStamped,
  Pose,
  Point,
  Quaternion,
)
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
import math


class ObjectTransform():
  def __init__(self):
    print("***__init__***")
    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bbox_callback)
    self.listener = tf.TransformListener()
    self.object_pub = rospy.Publisher('/object_pose', Pose, queue_size=10)
    self.object_pose = geometry_msgs.msg.Pose()

  def bbox_callback(self, darknet_bboxs):
    bboxs = darknet_bboxs.bounding_boxes
    bbox = BoundingBox()
    if len(bboxs) != 0:
        for i, bb in enumerate(bboxs):
            print(bboxs[i].Class)
            if bboxs[i].Class == "Noodle":
                (trans,rot) = self.listener.lookupTransform('/world', '/%s' % (bboxs[i].Class), rospy.Time(0))
                self.object_pose.position.x = trans[0]
                self.object_pose.position.y = trans[1]
                self.object_pose.position.z = trans[2]
                self.object_pose.orientation.x = rot[0]
                self.object_pose.orientation.y = rot[1]
                self.object_pose.orientation.z = rot[2]
                self.object_pose.orientation.w = rot[3] 
            else:
                self.object_pose.position.x = 0
                self.object_pose.position.y = 0
                self.object_pose.position.z = 0
                self.object_pose.orientation.x = 0
                self.object_pose.orientation.y = 0
                self.object_pose.orientation.z = 0
                self.object_pose.orientation.w = 0
            self.object_pub.publish(self.object_pose) 


if __name__ == '__main__':
    rospy.init_node('ar_transform') 
    ObjectTransform() 
    rospy.spin()
