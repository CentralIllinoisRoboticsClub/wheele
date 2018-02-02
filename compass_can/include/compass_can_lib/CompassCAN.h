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
        int convertCAN(const can_msgs::Frame& frame, float data_out[4], unsigned int nVars); //, unsigned int numBytesPerVar)
    
        void compassCallback(const can_msgs::Frame& frame);

        ros::NodeHandle nh_;
        ros::Publisher heading_pub_;
        ros::Publisher mag_pub_;
        ros::Subscriber can_sub_;
        
        std_msgs::Float32 heading;
        geometry_msgs::Vector3Stamped magXYZ;
};

#endif
