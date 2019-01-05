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

class WaypointManager():
    def __init__(self):        
        self.tf_listener = tf.TransformListener()
        
        rospy.init_node('waypoint_manager')
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.clicked_goal_callback, queue_size = 2)
        rospy.Subscriber('raw_cone_pose', PoseStamped, self.raw_cone_callback, queue_size=2)
        rospy.Subscriber('obs_cone_pose', PoseStamped, self.obs_cone_callback, queue_size=2)
        rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        
        self.wp_pub = rospy.Publisher('wp_goal', PoseStamped, queue_size=2)
        self.wp_cone_pub = rospy.Publisher('wp_cone_pose', PoseStamped, queue_size=2)
        self.found_cone_pub = rospy.Publisher('found_cone', Int16, queue_size=2)
        
        print('Initializing Waypoint Manager.')

        self.botx_odom = 0.0
        self.boty_odom = 0.0
        
        self.botx_map = 0.0
        self.boty_map = 0.0
        
        self.goal = PoseStamped()
        self.goal.header.frame_id = "odom"
        self.goal.pose.orientation.w = 1.0
        
        self.cur_wp = PoseStamped()
        self.cur_wp.header.frame_id = "odom"
        self.cur_wp.pose.orientation.w = 1.0
        
        self.map_wp = PoseStamped()
        self.map_wp.header.frame_id = "map"
        self.map_wp.pose.orientation.w = 1.0
        
        waypoints  = np.array([
          [lon_start, lat_start],
          [lon1, lat1],
          [lon2, lat2] ])
           
        self.num_waypoints = len(waypoints)
        
        [lon0,lat0] = waypoints[0]
        deg2meters = 111111.11
        lat_factor = math.cos(lat0*3.14159/180.0)
        self.wp_map = np.zeros(waypoints.shape)
        self.wp_map[:,0] = deg2meters*(waypoints[:,0]-lon0)
        self.wp_map[:,1] = deg2meters*lat_factor*(waypoints[:,1]-lat0)
        
        self.wp_k = 1
        time.sleep(1.0)
        self.update_waypoint()
    
    def update_waypoint(self):
        print "Update Waypoint"
        if(self.wp_k < self.num_waypoints):
            self.map_wp.header.stamp = rospy.Time.now()
            self.map_wp.pose.position.x = self.wp_map[self.wp_k,0]
            self.map_wp.pose.position.y = self.wp_map[self.wp_k,1]
            print "map_wp"
            print self.map_wp
            p_in_odom = self.xy_in_odom(self.map_wp)
            print "p in odom"
            print p_in_odom
            if(p_in_odom):
                self.cur_wp = p_in_odom
                self.wp_k += 1
                print "wp_goal published"
                self.wp_pub.publish(self.cur_wp)
        
    def odom_callback(self,odom):
        self.botx_odom = odom.pose.pose.position.x
        self.boty_odom = odom.pose.pose.position.y
        
    def clicked_goal_callback(self,data):
        print "Clicked Goal Callback"
        if(data.header.frame_id != "odom"):
            p_in_odom = self.xy_in_odom(data)
            if(p_in_odom):
                self.wp_pub.publish(p_in_odom)
        else:
            self.wp_pub.publish(data)
            
    def raw_cone_callback(self, data):
        #print "Raw Cone Callback"
        p_in_odom = self.xy_in_odom(data)
        if(p_in_odom):
            self.wp_cone_pub.publish(p_in_odom)
    
    def obs_cone_callback(self, data):
        self.cur_wp = data
        self.wp_pub.publish(self.cur_wp)
        msg = Int16()
        msg.data = 1
        self.found_cone_pub.publish(msg)
        #print("Waypoint Manager Received nearest obstacle to cone")
    
    def xy_in_odom(self, poseStamped):
        src_frame = poseStamped.header.frame_id
        p_in_odom = None
        count = 0
        try:
            self.tf_listener.waitForTransform("odom", src_frame, rospy.Time.now(), rospy.Duration(1.0))
            p_in_odom = self.tf_listener.transformPose("odom", poseStamped)
        except:
            rospy.logwarn("Error converting to odom frame")
            p_in_odom = None
        #~ while(not p_in_odom and count < 10):
            #~ count += 1
            #~ try:
                #~ #t = self.tf.getLatestCommonTime("/odom", src_frame)
                #~ p_in_odom = self.tf_listener.transformPose("odom", poseStamped)
            #~ except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                #~ p_in_odom = None
        #~ if(not p_in_odom):
            #~ rospy.logwarn("Error converting to odom frame")
        
        return p_in_odom

if __name__ == '__main__':
    try:
        wp_man = WaypointManager()
        print("Starting Waypoint Manager")

        r = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            r.sleep()
            
    except rospy.ROSInterruptException:
        pass
