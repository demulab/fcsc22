#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This import is for general library
import os
import threading
import random

# This import is for ROS integration
import rospy
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
import cv2
import math
import tf
from bounding_to_position.msg import object_pose

#camera param
HFOV = math.radians(47.0)
VFOV = math.radians(54.2) 
WIDTH = 640
HEIGHT = 480

class BoundingToPosition():
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.bbox = BoundingBox() 
        self.tb = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
    
        self.oj_pose = object_pose()
        self.object_pub = rospy.Publisher('/object_pose', object_pose, queue_size=10) 
        rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bbox_callback)

    def rgb_callback(self, rgb_image_data):
        try:
            rgb_image = self.cv_bridge.imgmsg_to_cv2(rgb_image_data)
        except CvBridgeError, e:
            rospy.logerr(e)
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)

    def depth_callback(self, depth_image_data):
        try:
            self.m_depth_image = self.cv_bridge.imgmsg_to_cv2(depth_image_data)
        except CvBridgeError, e:
            rospy.logerr(e)
        self.m_camdepth_height, self.m_camdepth_width = self.m_depth_image.shape[:2]

    def bbox_to_xyz(self):
        px = (int)(self.bbox.xmax + self.bbox.xmin) / 2
        py = (int)(self.bbox.ymax + self.bbox.ymin) / 2
        m_depth = self.m_depth_image[py][px]
        h_angle = -(px - WIDTH / 2) * (HFOV / WIDTH)
        v_angle = -(py - HEIGHT / 2) * (VFOV / HEIGHT)
        x = m_depth * math.tan(v_angle) * 0.001
        y = m_depth * math.tan(h_angle) * 0.001
        z = m_depth * 0.001
        print(self.bbox.Class, self.bbox.probability, x, y, z)
        self.tb.sendTransform((z, y, x), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), self.bbox.Class, "camera_link")

        time = rospy.Time.now()
        self.oj_pose.pose.header.stamp = time
        self.oj_pose.pose.header.frame_id = "camera_link"
        self.oj_pose.object_name = self.bbox.Class
        (trans,rot) = self.listener.lookupTransform('/world', '/%s' % (self.bbox.Class), rospy.Time(0))
        self.oj_pose.pose.pose.position.x = trans[0]
        self.oj_pose.pose.pose.position.y = trans[1]
        self.oj_pose.pose.pose.position.z = trans[2]
        self.oj_pose.pose.pose.orientation.x = rot[0]
        self.oj_pose.pose.pose.orientation.y = rot[1]
        self.oj_pose.pose.pose.orientation.z = rot[2]
        self.oj_pose.pose.pose.orientation.w = rot[3]
        self.object_pub.publish(self.oj_pose)

    def bbox_callback(self, darknet_bboxs):
        bboxs = darknet_bboxs.bounding_boxes
        bbox = BoundingBox()
        if len(bboxs) != 0:
            print(bboxs)
            random.shuffle(bboxs)
            print(bboxs)
            for i, bb in enumerate(bboxs):
                if bboxs[i].Class == 'Tomato Soup':
                    self.bbox = bboxs[i]
                elif bboxs[i].Class == 'Green tea':
                    self.bbox = bboxs[i]
                elif bboxs[i].Class == 'Chocolate Snack':
                    self.bbox = bboxs[i]
                elif bboxs[i].Class == 'Detergent':
                    self.bbox = bboxs[i]
                elif bboxs[i].Class == 'Refresher':
                    self.bbox = bboxs[i]
                elif bboxs[i].Class == 'Scrubbing brush':
                    self.bbox = bboxs[i]
                elif bboxs[i].Class == 'Cooling sheet':
                    self.bbox = bboxs[i]
                elif bboxs[i].Class == 'Bond':
                    self.bbox = bboxs[i]
                elif bboxs[i].Class == 'Plaster':
                    self.bbox = bboxs[i]
                self.bbox_to_xyz()
if __name__ == '__main__':
    try:
        rospy.init_node('bounding_to_position', anonymous=True)
        print("bounding_to_position")
        BoundingToPosition()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
