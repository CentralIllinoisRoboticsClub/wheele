// Copyright 2019 coderkarl. Subject to the BSD license.

#ifndef ImuCAN_H
#define ImuCAN_H

//ROS Includes
#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Vector3.h"
#include <vector>
//#include "std_msgs/Float64.h"
//#include <std_msgs/String.h>


class ImuCAN
{
    public:
        ImuCAN();
        ~ImuCAN();
        
    private:
        int convertCAN(const can_msgs::Frame& frame, float data_out[4], unsigned int nVars); //, unsigned int numBytesPerVar)
    
        void imuCallback(const can_msgs::Frame& frame);

        ros::NodeHandle nh_;
        ros::Publisher imu_pub_;
        ros::Publisher mag_pub_;
        ros::Publisher heading_pub_;
        ros::Subscriber can_sub_;
        
        sensor_msgs::Imu imu;
        sensor_msgs::MagneticField magXYZ;
        sensor_msgs::Imu mag_imu;
        
        geometry_msgs::Vector3 accel;
        //std::vector<double> accel_covar; //What is the right type for ROS float64[9]?? boost::array< >?
        //std::vector<double> gyro_covar;
        //std::vector<double> mag_covar;
        //std::vector<double> quat_covar;
        
        float compass_heading;
};

#endif
