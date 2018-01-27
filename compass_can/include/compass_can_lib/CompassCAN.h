#ifndef CompassCAN_H
#define CompassCAN_H

//ROS Includes
#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float32.h"
//#include <std_msgs/String.h>


class CompassCAN
{
    public:
        CompassCAN();
        ~CompassCAN();
        
    private:
        void compassCallback(const can_msgs::Frame& frame);
        //void leddarCallback(const can_msgs::Frame::ConstPtr& frame);


        ros::NodeHandle nh_;
        ros::Publisher heading_pub_;
        ros::Subscriber can_sub_;
        
        uint8_t num_detections; //number of detections given in 0x751
        uint8_t det_count; //counter to check we have the expected scan data
        
        std_msgs::Float32 heading;
};

#endif
