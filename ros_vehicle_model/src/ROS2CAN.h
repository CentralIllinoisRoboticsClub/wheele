// Copyright 2019 coderkarl. Subject to the BSD license.

#ifndef ROS2CAN_H
#define ROS2CAN_H

//ROS Includes
#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <geometry_msgs/Twist.h>
//#include <std_msgs/String.h>

#include <stdint.h>


class ROS2CAN
{
    public:
        ROS2CAN();
        ~ROS2CAN();
        
        //returns parameter rate_ in hz
        // used to define ros::Rate
        int get_rate();
        void sendCmdVel();
        
    private:
        void cmdVelCallback(const geometry_msgs::Twist& cmd);

        void tx_can(uint16_t id, int16_t num1, int16_t num2, int16_t num3, int16_t num4);
        uint8_t getByte(uint16_t x, unsigned int n);

        ros::NodeHandle nh_;
        ros::NodeHandle nh_p;
        ros::Publisher canFrame_pub_;
        ros::Subscriber cmd_sub_;
        
        geometry_msgs::Twist latest_cmd_vel;
        can_msgs::Frame canFrame;
        
        ros::Time cmd_vel_time;

        //parameters
        int rate_;
        double timeout_sec_;
};

#endif
