#!/usr/bin/env python

import rospy, math
import numpy as np
from wheele_msgs.msg import SpeedCurve

from std_msgs.msg import Int16

from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Twist, Quaternion, Point, Pose, Vector3, Vector3Stamped
from sensor_msgs.msg import Imu

import time
import can
import sys
import binascii

class CANConverter():
    def __init__(self):
        self.roll_rad = 0
        self.pitch_rad = 0
        self.N_roll = 0
        self.heading_init_count = 0
        self.mag_heading_deg = 0
        self.odom_heading_deg = 0
        self.map_bc = tf.TransformBroadcaster()
        
        self.tf_listener = tf.TransformListener()
        
        rospy.init_node('can_converter')
        rospy.Subscriber('imu', Imu, self.accelIMU_callback, queue_size=2)
        rospy.Subscriber('mag_imu', Imu, self.magIMU_callback, queue_size=2)
        rospy.Subscriber('gps_pose',Vector3Stamped, self.gps_callback, queue_size=5)
        
        self.cmd_pub = rospy.Publisher('wheele_cmd_man', SpeedCurve, queue_size=1)
        self.batt_pub = rospy.Publisher('wheele_batt', Int16, queue_size = 1)
        self.auto_raw_pub = rospy.Publisher('auto_raw', Int16, queue_size = 1)
        self.raw_cmd_pub = rospy.Publisher('raw_cmd_py', Vector3, queue_size = 10)
        self.raw_cmd = Vector3()
        self.cmd = SpeedCurve()
        self.cmd.speed = 0.0
        self.cmd.curvature = 0.0
        
        self.bad_raw_cmd_count = 0
        
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=5)
        #self.odom_ekf_pub = rospy.Publisher('odom_ekf', Odometry, queue_size=5)
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        self.prev_time = rospy.Time.now()
        
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan_ctypes')
        # http://python-can.readthedocs.io/en/latest/interfaces/socketcan.html
        
        #Also publish odom like in jeep_ros_comm.py
        #For now, just use encoder data for odom
        #need to add IMU to micro, send IMU data over CAN
        #Consider just publishing the encoder data here
        #   Another node will publish odom

        #Subscribe to ThrustSteer 'thrust_and_steer' published by Vehicle Model
        #  Send this data over CAN to micro that controls micro maestro or servos/ESCs directly
        
        self.raw_battery_voltage = -1
        
        self.raw_speed_cmd = 0
        self.raw_steer_cmd = 0
        self.raw_auto_cmd = 0
        self.man_count = 0
        self.auto_stop_flag = True
        
        self.gyroz_degx100 = 0
        
        self.left_enc = 0
        self.right_enc = 0
        self.prev_left_enc = -1
        self.prev_right_enc = -1
        self.enc_init_flag = False
        print('Initializing can converter.')
        print('REMEMBER: sudo ip link set can0 up type can bitrate 500000')
        
        while(not self.enc_init_flag):
            self.update_CAN()
            self.prev_left_enc = self.left_enc
            self.prev_right_enc = self.right_enc
        
        self.dist_sum = 0
        self.time_sum = 0
        self.vx = 0

        self.bot_deg = 0
        self.botx = 0
        self.boty = 0

    def gps_callback(self,data):
        x = data.vector.x
        y = data.vector.y
        gps_theta = math.atan2(y,x)
        (trans,quat) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        xb = trans[0]
        yb = trans[1]
        if(self.botx**2 + self.boty**2 > 20.**2):
            print "Updating odom heading based on gps"
            bot_theta = math.atan2(yb,xb)
            diff_theta = gps_theta - bot_theta
            self.odom_heading_deg += diff_theta*180./3.14
        

    def magIMU_callback(self, data):
        quaternion = (data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w)
        [roll,pitch,yaw] = tf.transformations.euler_from_quaternion(quaternion)
        self.mag_heading_deg = yaw*180./3.14159
        if(self.heading_init_count < 10):
            self.heading_init_count += 1
            if(self.heading_init_count == 1):
                self.odom_heading_deg = self.mag_heading_deg
            else:
                self.odom_heading_deg = (self.odom_heading_deg*(self.heading_init_count-1) + self.mag_heading_deg)/self.heading_init_count
                print "init odom deg: ", self.odom_heading_deg
        # map to odom
        quat = tf.transformations.quaternion_from_euler(0, 0, self.odom_heading_deg*3.1416/180.0)
        self.map_bc.sendTransform(
        (0, 0, 0.),
        quat,
        data.header.stamp,
        "odom",
        "map"
        )
    def accelIMU_callback(self, data):
        accx = data.linear_acceleration.x
        accy = data.linear_acceleration.y
        ##### USE IMU TO PUBLISH TRANSFORM BETWEEN LASER AND BASE
        br = tf.TransformBroadcaster()
        if(abs(accx) < 3 and abs(accy) < 3):
            try:
                roll_rad = math.asin(accy/9.81) +0.0
                pitch_rad = math.asin(-accx/9.81) +0.0
            except:
                roll_rad = self.roll_rad
                pitch_rad = self.pitch_rad
                print('asin error for roll or pitch')
        else:
            roll_rad = self.roll_rad
            pitch_rad = self.pitch_rad
            print('accx,y above 3 m/s^2')
                
        self.roll_rad = 0.9*self.roll_rad + 0.1*roll_rad
        self.pitch_rad = 0.9*self.pitch_rad + 0.1*pitch_rad
        self.N_roll += 1
        if(self.N_roll >= 10):
            #print('roll rad: ', self.roll_rad, ', pitch rad: ', self.pitch_rad)
            self.N_roll = 0
        
        t2 = rospy.Time.now()
        laser_quat = tf.transformations.quaternion_from_euler(self.roll_rad, self.pitch_rad, 0)
        br.sendTransform((-0.3,0.1,0.3),laser_quat,t2,"laser","base_link")
        #####

    def update_CAN(self):
        msg = self.bus.recv(0.0)
        new_cmd_data_flag = False
        new_gyro_data_flag = False
        new_enc_data_flag = False
        new_all_data_flag = False
        read_count = 0
        #Does the code below run fast enough to ever actually clear the CAN buffer?
        #while(msg != None and  not new_all_data_flag):
        while(msg != None and not new_all_data_flag):
            if(msg.arbitration_id == 0x101): #RAW RC CMD, NOTE auto_cmd (left hz joystick) is not sent yet from wheele_ard_main.ino
                #print msg.arbitration_id, convertCAN(msg.data, 2, 2)
                self.raw_speed_cmd, self.raw_steer_cmd, self.raw_auto_cmd = self.convertCAN(msg.data,3,2)
                new_cmd_data_flag = True
                read_count = read_count + 1
                #print('read cmd')
            elif(msg.arbitration_id == 0x131): #GYRO
                gyro_list = self.convertCAN(msg.data,3,2)
                self.gyroz_degx100 = gyro_list[2]
                new_gyro_data_flag = True
                read_count = read_count + 1
                #print('read gyro')
            elif(msg.arbitration_id == 0x105): #ENCODERS
                self.left_enc, self.right_enc = self.convertCAN(msg.data,2,2)
                self.enc_init_flag = True
                new_enc_data_flag = True
                read_count = read_count + 1
                #print('read enc')
            elif(msg.arbitration_id == 0x140): #BATTERY
                self.raw_battery_voltage = self.convertCAN(msg.data,1,2)[0]
                self.pub_battery_signal()
                
            new_all_data_flag = new_cmd_data_flag and new_gyro_data_flag and new_enc_data_flag
            msg = self.bus.recv(0.0)
        #print 'CAN update done'
    
    def pub_battery_signal(self):
        raw_sig = Int16()
        raw_sig.data = self.raw_battery_voltage
        self.batt_pub.publish(raw_sig)
    
    def update_cmd(self):
        
        # Assume the bad raw speed, steer cmd data is not bad at the same time very often
        if(self.raw_speed_cmd < 900 or self.raw_speed_cmd > 2000): #This happens frequently when driving
            if(self.raw_steer_cmd < 900 or self.raw_steer_cmd > 2000): #This happens frequently when driving
                self.bad_raw_cmd_count += 1
                if(self.bad_raw_cmd_count > 15):
                    self.cmd.speed = 0
                    self.cmd.curvature = 0
            else:
                self.bad_raw_cmd_count = 0
        else:
            self.cmd.speed = (self.raw_speed_cmd-1350)*10.0/370.0 #raw command is 1350 +/- 370
            if(self.raw_steer_cmd > 900 and self.raw_steer_cmd < 2000):
                self.cmd.curvature = (self.raw_steer_cmd-1380)*2.5/370.0 #raw command is 1380 +/- 370
        if(math.fabs(self.cmd.speed) < 0.5):
            self.cmd.speed = 0
        
        self.cmd_pub.publish(self.cmd)
        
        self.raw_cmd.x = self.raw_speed_cmd
        self.raw_cmd.y = self.raw_steer_cmd
        self.raw_cmd.z = self.raw_auto_cmd
        self.raw_cmd_pub.publish(self.raw_cmd)
        
        auto_raw_msg = Int16()
        auto_raw_msg.data = self.raw_auto_cmd
        self.auto_raw_pub.publish(auto_raw_msg)
        #print 'Published cmd'      
                
    def update_odom(self):
        t2 = rospy.Time.now()
        t1 = self.prev_time
        self.prev_time = t2
        dt = (t2-t1).to_sec()
        
        gyro_thresh_dps = 0.00
        g_bias_dps = 0.0
        MAX_DTHETA_GYRO_deg = 100
        BOT_WIDTH = (23.0 * 2.54 / 100.0) #meters
        COUNTS_PER_METER = 900.0
        
        gyroz_raw_dps = float(self.gyroz_degx100) / 100.0
        #print 'gyroz raw dps: ', gyroz_raw_dps
        
        delta_left_enc = self.get_delta_enc(self.left_enc, self.prev_left_enc)
        delta_right_enc = self.get_delta_enc(self.right_enc, self.prev_right_enc)
        
        dtheta_enc_deg = float(delta_right_enc - delta_left_enc) / BOT_WIDTH * 180.0 / 3.1416
        
        dmeters = float(delta_left_enc + delta_right_enc)/2.0 / COUNTS_PER_METER #900 counts/meter
        #print 'dmeters: ', dmeters
        
        if(abs(gyroz_raw_dps+g_bias_dps) < gyro_thresh_dps):
            gz_dps = 0
            dtheta_gyro_deg = 0
        else:
            gz_dps = gyroz_raw_dps+g_bias_dps
            dtheta_gyro_deg = gz_dps*dt*360.0/368.2#*360.0/375.0 #HACK, WHY!!??

        if(abs(dtheta_gyro_deg) > MAX_DTHETA_GYRO_deg):
            print 'no gyro'
            dtheta_deg = dtheta_enc_deg
        else:
            #print 'use gyro'
            dtheta_deg = dtheta_gyro_deg
            
        #print 'dtheta gyro deg:', dtheta_gyro_deg
        #print 'dtheta enc deg:', dtheta_enc_deg

        #update bot position
        #self.bot.move(dmeters,dtheta_deg,use_gyro_flag)
        self.bot_deg = self.bot_deg + dtheta_deg
        dx = dmeters*np.cos(self.bot_deg*3.1416/180)
        dy = dmeters*np.sin(self.bot_deg*3.1416/180)
        self.botx = self.botx + dx
        self.boty = self.boty + dy
        
        # update bot linear x velocity every 150 msec
        # need to use an np array, then push and pop, moving average
        self.dist_sum = self.dist_sum + dmeters
        self.time_sum = self.time_sum + dt
        if(self.time_sum > 0.15):
            self.vx = self.dist_sum / self.time_sum
            self.dist_sum = 0
            self.time_sum = 0
        
        #bot.botx*100,bot.boty*100,bot.bot_deg
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.bot_deg*3.1416/180.0)
        self.odom_broadcaster.sendTransform(
        (self.botx, self.boty, 0.),
        odom_quat,
        t2,
        "base_link",
        "odom"
        )
        
        odom = Odometry()
        odom.header.stamp = t2
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.botx, self.boty, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(self.vx, 0, 0), Vector3(0, 0, gz_dps*3.1416/180.0))

        # odom twist covariance
        odom.twist.covariance[0] = 0.001 #vx
        odom.twist.covariance[7] = 1.e-9 #vy
        # publish the message
        self.odom_pub.publish(odom)
        
        #odom.header.frame_id = "odom_ekf"
        #odom.child_frame_id = "base_link_ekf"
        #self.odom_ekf_pub.publish(odom)        
        
        self.prev_left_enc = self.left_enc
        self.prev_right_enc = self.right_enc

    def get_delta_enc(self, cur, prev):
        delta = cur - prev
        if(delta > 60000):
            delta = -(65535 - delta)
        elif(delta < -60000):
            delta = 65535 + delta
        return delta

    # Custom CAN conversion, varSize is in # bytes per variable
    # Is it faster to just assume either 2 byte vars or 3 byte vars for our protocol?
    #   then replace the - (256**varSize)/2 with if varSize, then -32768 or -8388608
    #   If we use 4 byte vars, do we have data type issues in python?
    #   Enter 256**4 / 2 into python, observe xxxL
    def convertCAN(self, data, nVars, varSize):
        y = []
        b = 0
        for k in range(nVars):
            #8*varSize will not always work. int(xx,8) is invalid. int(xx,24) may be invalid
            #    if we want int(xx,8), just do int(xx,16), so perhaps we can always just do the max # bits? 32?
            y.append( int(binascii.hexlify(data[b:b+varSize]),8*varSize) - (256**varSize)/2)
            b = b+varSize
        return y

if __name__ == '__main__':
    try:
        can_conv = CANConverter()
        print("Starting can_converter")

        r = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            can_conv.update_CAN()
            can_conv.update_cmd()
            can_conv.update_odom()
            r.sleep()
            
    except rospy.ROSInterruptException:
        pass
