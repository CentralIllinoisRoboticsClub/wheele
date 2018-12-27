#!/usr/bin/env python

import rospy, math
import numpy as np

from std_msgs.msg import Int16

import tf
from geometry_msgs.msg import Quaternion, Point, Pose, PoseStamped, Vector3, Vector3Stamped
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry

import time
import sys

class ConeCoordinates():
    def __init__(self):        
        self.tf_listener = tf.TransformListener()
        
        rospy.init_node('cone_coordinates')
        rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        
        self.base_cone_pub = rospy.Publisher('base_cone_pose', PoseStamped, queue_size=2)
        
        print('Initializing Cone Coordinates.')

        self.botx_odom = 0.0
        self.boty_odom = 0.0
        
        self.cone = PoseStamped()
        self.cone.header.frame_id = "odom"
        self.cone.pose.orientation.w = 1.0
        self.cone.pose.position.x = 7.3
        self.cone.pose.position.y = -3
        
    def odom_callback(self,odom):
        self.botx_odom = odom.pose.pose.position.x
        self.boty_odom = odom.pose.pose.position.y
        c = self.cone_in_base()
        if(c):
            self.base_cone_pub.publish(c)
    
    def cone_in_base(self):
        src_frame = "odom"
        p_in_base = None
        count = 0
        try:
            self.tf_listener.waitForTransform("base_link", src_frame, rospy.Time.now(), rospy.Duration(1.0))
            p_in_base = self.tf_listener.transformPose("base_link", self.cone)
        except:
            rospy.logwarn("Error converting to base frame")
            p_in_base = None
        
        return p_in_base

if __name__ == '__main__':
    try:
        cone_coords = ConeCoordinates()
        print("Starting Cone Coordinates")

        r = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            r.sleep()
            
    except rospy.ROSInterruptException:
        pass
