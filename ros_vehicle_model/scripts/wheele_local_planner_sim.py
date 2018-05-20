#!/usr/bin/env python

import rospy, math
import numpy as np
from geometry_msgs.msg import Twist
from wheele_msgs.msg import SpeedCurve
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf

from DiffDriveController import DiffDriveController

class PathController():
    def __init__(self):
        rospy.init_node('path_controller')
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        #rospy.Subscriber('cmd_vel', Twist, self.drive_callback, queue_size=1)
        rospy.Subscriber('auto_mode', Int16, self.auto_mode_callback, queue_size = 1)
        rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size = 1)
        rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.path_callback, queue_size = 1)
        #<remap from="topic_a_temp" to="/ns1/topic_a">
        
        self.auto_mode = 1700
        self.vx = 0.0
        self.cum_err = 0
        
        MAX_SPEED = 2.0
        MAX_OMEGA = 3.0
        self.diff_drive_controller = DiffDriveController(MAX_SPEED, MAX_OMEGA)
        
        x_i = 0.
        y_i = 0.
        theta_i = -99.
        self.state = np.array([[x_i], [y_i], [theta_i] ])
        self.goal = np.array([[x_i], [y_i] ])
        self.goal_reached = True
        
        self.waypoints = []
        self.v = 0.0
        self.w = 0.0
        
        self.tf_listener = tf.TransformListener()
    
    def auto_mode_callback(self, data):
        self.auto_mode = data
    
    def odom_callback(self,odom):
        # This odom pose is in the odom frame, we want the pose in the map frame
        #   Now done in execute_plan() at a fixed rate using a tf.TransformListener()
        #~ quat = odom.pose.pose.orientation
        #~ quat_list = [quat.x, quat.y, quat.z, quat.w]
        #~ (roll, pitch, yaw) = euler_from_quaternion (quat_list)
        
        #~ self.state[0] = odom.pose.pose.position.x + 5.0
        #~ self.state[1] = odom.pose.pose.position.y
        #~ self.state[2] = yaw
        
        self.vx = odom.twist.twist.linear.x
    
    def path_callback(self,data):
        poses = data.poses
        nPose = len(poses)
        self.waypoints = []
        if(nPose > 0):
            wp = np.zeros([2,1])
            if(nPose == 1):
                init = 0
                wp_step = 1
            else:
                # approximate actual path length by breaking into 4 sub-linesegments
                path_dist = 0
                nSegments = 4
                stride = max(int(nPose/nSegments), 1)
                ind1 = 0
                ind2 = 0
                done_flag = False
                k = 0
                while(k <= nSegments):
                    ind2 += stride
                    if(ind2 >= nPose):
                        ind2 = nPose -1
                        done_flag = True
                    dx = poses[ind2].pose.position.x - poses[ind1].pose.position.x
                    dy = poses[ind2].pose.position.y - poses[ind1].pose.position.y
                    path_dist += np.sqrt(dx**2 + dy**2)
                    if(done_flag):
                        print "reached final point for path_dist"
                        break
                    ind1 = ind2
                    k += 1
                    
                wp_step = max(int(path_dist/2.0), 1)
                init = wp_step
            for k in xrange(init,nPose,wp_step):
                wp[0] = poses[k].pose.position.x
                wp[1] = poses[k].pose.position.y
                self.waypoints.append(wp.copy())
            if ( not k==nPose-1):
                wp[0] = poses[-1].pose.position.x
                wp[1] = poses[-1].pose.position.y
                self.waypoints.append(wp.copy())
            
            #print "waypoints: ", self.waypoints
            self.goal = self.waypoints.pop(0)
            print "initial goal: ", self.goal
            
            self.goal_reached = False
        else:
            self.goal_reached = True
            self.waypoints = []
            
        
    def execute_plan(self):
        # Update robot pose state from tf listener
        try:
            (trans,quat) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            self.state[0] = trans[0]
            self.state[1] = trans[1]
            #quat_list = [quat.x, quat.y, quat.z, quat.w]
            (roll, pitch, yaw) = euler_from_quaternion (quat)
            self.state[2] = yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        
        # Local planner (no dynamic obstacle avoidance, hopefully to be done by global planner called frequently)
        if(not self.goal_reached):
            self.v,self.w,self.goal_reached, alpha, pos_beta = self.diff_drive_controller.compute_vel(self.state,self.goal)
            if(self.goal_reached):
                print "wp goal reached"
                #print "v: ", v
                #print "w: ", w
                print "state: ", self.state
                print "goal: ", self.goal
        elif( (self.goal_reached or abs(alpha) > 3.14/2) and len(self.waypoints) > 0):
            self.goal = self.waypoints.pop(0)
            print "wp goal: ", self.goal
            self.goal_reached = False
        else:
            self.v = 0.
            self.w = 0.
        
        #PI control for desired linear speed v
        # controller will output an offset command to add to v
        Ks = 2.5
        Ki = 1.5
        Kp = 1.5
        err = self.v - self.vx
        self.cum_err += err
        self.cum_err = min(1.0, self.cum_err)
        self.cum_err = max(-1.0, self.cum_err)
        out = Kp*err + Ki*self.cum_err
        
        twist = Twist()
        twist.linear.x = self.v #Ks*v+out
        twist.angular.z = self.w
        self.cmd_pub.publish(twist)
        
        #if(self.auto_mode > 1600):
        #self.cmd_pub.publish(spdCrv)

if __name__ == '__main__':
    try:
        path_control = PathController()
        rospy.loginfo("Starting Wheele Path Controller")
        #rospy.spin()
        r = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            path_control.execute_plan()
            r.sleep()
            
    except rospy.ROSInterruptException:
        pass
