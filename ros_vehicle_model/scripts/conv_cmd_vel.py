#!/usr/bin/env python

import rospy, math
import numpy as np
from geometry_msgs.msg import Twist
from wheele_msgs.msg import SpeedCurve
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry

class CMDConverter():
    def __init__(self):
        rospy.init_node('cmd_converter')
        self.cmd_pub = rospy.Publisher('wheele_cmd_auto', SpeedCurve, queue_size=1)
        rospy.Subscriber('cmd_vel', Twist, self.drive_callback, queue_size=1)
        rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size = 1)
        
        self.vx = 0.0
        self.cum_err = 0
    
    def odom_callback(self,odom):
        self.vx = odom.twist.twist.linear.x
        
    def drive_callback(self, data):
        spdCrv = SpeedCurve()
        v = data.linear.x
        w = data.angular.z
        MAX_CURVATURE = 2.0
        MIN_SPEED = 1.0
        
        if(abs(v) > 0.0):
            curv = w/v
            if(abs(v) < MIN_SPEED):
                v = np.sign(v)*MIN_SPEED
        else:
            curv = np.sign(w)*MAX_CURVATURE
            if(abs(curv) > 0.0):
                v = MIN_SPEED
        
        if(abs(curv) > MAX_CURVATURE):
            curv = np.sign(curv)*MAX_CURVATURE
        
        spdCrv.curvature = curv
        
        #PI control for desired linear speed v
        # controller will output an offset command to add to v
        Ks = 2.5
        Ki = 1.5
        Kp = 1.5
        err = v - self.vx
        self.cum_err += err
        self.cum_err = min(1.0, self.cum_err)
        self.cum_err = max(-1.0, self.cum_err)
        out = Kp*err + Ki*self.cum_err
        
        spdCrv.speed = Ks*v + out
        
        self.cmd_pub.publish(spdCrv)

if __name__ == '__main__':
    try:
        cmd_conv = CMDConverter()
        print("Starting cmd_vel converter")
        rospy.spin()
            
    except rospy.ROSInterruptException:
        pass
