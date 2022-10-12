#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
import math

def main():
    rospy.init_node("ar_debug")
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, cb)


def cb(ar):
    for d in ar.markers:
        ox = d.pose.pose.orientation.x
        oy = d.pose.pose.orientation.y
        oz = d.pose.pose.orientation.z
        ow = d.pose.pose.orientation.w
        rot = (ox, oy, oz, ow)
        euler = tf.transformations.euler_from_quaternion(rot)
        degree = (euler[0]*180/math.pi ,euler[1]*180/math.pi, euler[2]*180/math.pi) 
        #print(degree)
   

def get_optimul_ar(ids):
    prev_x = 180
    prev_y = 180
    print("goa")
    listener = tf.TransformListener()
    for data in ids:
        print("######################################################")
        print("processing", str(data))
        try:
            (trans, rot) = listener.lookupTransform("/world", "/%s" % ("ar_marker_" + str(data)), rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            ex = abs(euler[0] * (180 / math.pi))
            ey = abs(euler[1] * (180 / math.pi))
            if ex < 45 and ey < 45:
                if ex <= prev_x:
                    if ey <= prev_y:
                        print(ex, ey)
                        prev_x = ex
                        prev_y = ey
                        opt_x = ex
                        opt_y = ey
                        #self.vacuum_picking(str(data))
                        print("id_", str(data), "_success!!!")
                        break
            else:
                pass
        except:
            pass



if __name__ == "__main__":
    try:
        main()
        ids = range(1, 7)
        get_optimul_ar(ids)

    except rospy.ROSInterruptException:
        pass

    
