#!/usr/bin/env python

import rospy, math
import numpy as np
from wheele_msgs.msg import SpeedCurve

import time
import can
import sys
import binascii

class CANConverter():
    def __init__(self):
        rospy.init_node('can_converter')
        self.cmd_pub = rospy.Publisher('wheele_cmd_vel', SpeedCurve, queue_size=1)
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan_ctypes')
        #Also publish odom like in jeep_ros_comm.py
        #For now, just use encoder data for odom
        #need to add IMU to micro, send IMU data over CAN
        #Consider just publishing the encoder data here
        #   Another node will publish odom

        #Subscribe to ThrustSteer 'thrust_and_steer' published by Vehicle Model
        #  Send this data over CAN to micro that controls micro maestro or servos/ESCs directly


    def update_cmd(self):
        msg = self.bus.recv(0.0)
        if(msg != None):
            #print msg.arbitration_id, convertCAN(msg.data,4,2)
            if(msg.arbitration_id == 0x101):
                #print msg.arbitration_id, convertCAN(msg.data, 2, 2)
                raw_speed, raw_steer = self.convertCAN(msg.data,2,2)
                cmd = SpeedCurve()
                if(raw_speed < 900 or raw_speed > 2000):
                    cmd.speed = 0
                elif(raw_steer < 900 or raw_steer > 2000):
                    cmd.curvature = 0
                else:
                    cmd.speed = (raw_speed-1350)*6.0/370.0
                    cmd.curvature = (raw_steer-1380)*2.5/370.0
                if(math.fabs(cmd.speed) < 0.5):
                    cmd.speed = 0
                    
                self.cmd_pub.publish(cmd)

    # Custom CAN conversion, varSize is in # bytes per variable
    # Is it faster to just assume either 2 byte vars or 3 byte vars for our protocol?
    #   then replace the - (256**varSize)/2+1 with if varSize, then -32767 or -8388607
    #   If we use 4 byte vars, do we have data type issues in python?
    #   Enter 256**4 / 2 - 1 into python, observe xxxL
    def convertCAN(self, data, nVars, varSize):
        y = []
        b = 0
        for k in range(nVars):
            y.append( int(binascii.hexlify(data[b:b+varSize]),8*varSize) - (256**varSize)/2+1)
            b = b+varSize
        return y

if __name__ == '__main__':
    try:
        can_conv = CANConverter()
        print("Starting can_converter")

        r = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            can_conv.update_cmd()
            r.sleep()
            
    except rospy.ROSInterruptException:
        pass
