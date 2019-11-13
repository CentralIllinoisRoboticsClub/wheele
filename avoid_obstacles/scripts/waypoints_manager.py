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
        
        self.near_goal_distance_threshold = 3.0
        self.cone_transition = False
        
        rospy.init_node('waypoint_manager')
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.clicked_goal_callback, queue_size = 2)
        rospy.Subscriber('raw_cone_pose', PoseStamped, self.raw_cone_callback, queue_size=2)
        rospy.Subscriber('obs_cone_pose', PoseStamped, self.obs_cone_callback, queue_size=2)
        rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber('bump_switch', Int16, self.bump_switch_callback, queue_size=1)
        
        self.wp_pub = rospy.Publisher('wp_goal', PoseStamped, queue_size=2)
        self.wp_cone_pub = rospy.Publisher('wp_cone_pose', PoseStamped, queue_size=2)
        self.found_cone_pub = rospy.Publisher('found_cone', Int16, queue_size=2)
        
        print('Initializing Waypoint Manager.')

        self.bump_switch = 0
        self.found_cone = False

        self.botx_odom = 0.0
        self.boty_odom = 0.0
        
        self.botx_map = 0.0
        self.boty_map = 0.0
        
        self.bot_pose_odom = PoseStamped()
        self.bot_pose_odom.header.frame_id = "odom"
        self.bot_pose_odom.pose.orientation.w = 1.0
        
        self.goal = PoseStamped()
        self.goal.header.frame_id = "odom"
        self.goal.pose.orientation.w = 1.0
        
        self.cur_wp = PoseStamped()
        self.cur_wp.header.frame_id = "odom"
        self.cur_wp.pose.orientation.w = 1.0
        
        self.map_wp = PoseStamped()
        self.map_wp.header.frame_id = "map"
        self.map_wp.pose.orientation.w = 1.0
        
        deg2meters = 111111.11
        meters2deg = 1.0/deg2meters

        #longitude, latitude
        #x,y +x=east, +y=north
        waypoints  = np.array([
          [0, 0],  #lon, lat start
          [4.0*meters2deg, -2.0*meters2deg],  #first cone
          [9.0*meters2deg, 0.0*meters2deg],  #second cone
          [9.0*meters2deg, 6.0*meters2deg] ])  #third cone
           
        self.num_waypoints = len(waypoints)
        
        [lon0,lat0] = waypoints[0]
        
        lat_factor = math.cos(lat0*3.14159/180.0)
        self.wp_map = np.zeros(waypoints.shape)
        self.wp_map[:,0] = deg2meters*(waypoints[:,0]-lon0)
        self.wp_map[:,1] = deg2meters*lat_factor*(waypoints[:,1]-lat0)
        
        self.wp_k = 1
        time.sleep(1.0)
        self.update_waypoint()
        self.touch_cone_time = 0
        

    def update_waypoint(self):
        print "Update Waypoint"
        #if(self.wp_k < self.num_waypoints):
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
            if(self.wp_k == self.num_waypoints):
                self.wp_k = 1
            print "wp_goal published"
            self.wp_pub.publish(self.cur_wp)
            self.wp_cone_pub.publish(p_in_odom) # publish this once to avoid obs so it will clear the costmap at the initial goal, until it finds an updated goal from the camera
        
    def bump_switch_callback(self,sw):
        #if(not self.cone_transition): # covered in main, we may want to know when the bump_switch is released
        self.bump_switch = sw.data

    def odom_callback(self,odom):
        self.botx_odom = odom.pose.pose.position.x
        self.boty_odom = odom.pose.pose.position.y
        self.bot_pose_odom = odom.pose
        
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
        p_in_map = self.xy_in_map(data)
        if(p_in_map and (self.distance_between_poses(p_in_map, self.map_wp) < self.near_goal_distance_threshold)):
            p_in_odom = self.xy_in_odom(data)
            if(p_in_odom):
                self.wp_cone_pub.publish(p_in_odom)
                # WARNING, THIS CAN BE PUBLISHED JUST BEFORE update_waypoint updates self.map_wp
                #    THIS WILL CAUSE A FUTURE obs_cone_callback AFTER update_waypoint, and self.cur_wp will be reverted
    
    def obs_cone_callback(self, data):
        # ENSURE prev cone pose does not revert self.cur_wp if a raw cone pose was sent to avoid obs just before update_waypoint
        if(not self.cone_transition and self.distance_between_poses(data, self.cur_wp) < self.near_goal_distance_threshold):
            self.cur_wp = data
            self.wp_pub.publish(self.cur_wp)
            msg = Int16()
            msg.data = 1
            self.found_cone_pub.publish(msg)
            self.found_cone = True
            #print("Waypoint Manager Received nearest obstacle to cone")
    
    def distance_between_poses(self, pose1, pose2):
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        dist_sqd = dx*dx + dy*dy
        return np.sqrt(dist_sqd)
    
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
        
        return p_in_odom
    
    def xy_in_map(self, poseStamped):
        src_frame = poseStamped.header.frame_id
        p_in_map = None
        count = 0
        try:
            self.tf_listener.waitForTransform("map", src_frame, rospy.Time.now(), rospy.Duration(1.0))
            p_in_map = self.tf_listener.transformPose("map", poseStamped)
        except:
            rospy.logwarn("Error converting to map frame")
            p_in_map = None
        
        return p_in_map

if __name__ == '__main__':
    try:
        wp_man = WaypointManager()
        print("Starting Waypoint Manager")

        r = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            if(wp_man.cone_transition):
                if( (rospy.Time.now() - wp_man.touch_cone_time).to_sec() > 5.0):
                    wp_man.cone_transition = False
            else:
                dist = wp_man.distance_between_poses(wp_man.cur_wp, wp_man.bot_pose_odom)
                if ( (wp_man.bump_switch and dist < 1.0) or (dist < 1.0 and not wp_man.found_cone) or (dist < 0.7 and wp_man.found_cone) ):
                    wp_man.cone_transition = True
                    wp_man.touch_cone_time = rospy.Time.now()
                    print dist
                    print wp_man.bump_switch
                    msg = Int16()
                    msg.data = 0
                    wp_man.found_cone_pub.publish(msg) #Now wheele_local_planner will see current cone as obstacle and back up
                    wp_man.found_cone = False
                    wp_man.update_waypoint()
            
            r.sleep()
            
    except rospy.ROSInterruptException:
        pass
