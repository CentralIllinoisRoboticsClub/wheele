// Copyright 2019 coderkarl. Subject to the BSD license.

#ifndef Can2Ros_H
#define Can2Ros_H

//ROS Includes
#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Vector3.h"
#include "wheele_msgs/Encoder.h"
#include <vector>
//#include "std_msgs/Float64.h"
//#include <std_msgs/String.h>

class Can2Ros
{
public:
  Can2Ros();
  ~Can2Ros();
  //returns parameter rate_ in hz
  // used to define ros::Rate
  int get_rate();
  void sendCmdVel();

private:
  int convertCAN(const can_msgs::Frame &frame, int16_t data_out[4],
      unsigned int nVars); //, unsigned int numBytesPerVar)

  void canCallback(const can_msgs::Frame &frame);
  
  void cmdVelCallback(const geometry_msgs::Twist& cmd);
  void tx_can(uint16_t id, int16_t num1, int16_t num2, int16_t num3, int16_t num4);
  uint8_t getByte(uint16_t x, unsigned int n);
        
  ros::NodeHandle nh_;
  ros::NodeHandle nh_p;
  ros::Publisher imu_pub_;
  ros::Publisher mag_pub_;
  ros::Publisher heading_pub_;
  ros::Publisher enc_pub_;
  ros::Publisher canFrame_pub_;

  ros::Subscriber can_sub_;
  ros::Subscriber cmd_sub_;
        
  sensor_msgs::Imu imu;
  sensor_msgs::MagneticField magXYZ;
  sensor_msgs::Imu mag_imu;
  wheele_msgs::Encoder enc;
  can_msgs::Frame canFrame;
  ros::Time cmd_vel_time;

  geometry_msgs::Vector3 accel;
  geometry_msgs::Twist latest_cmd_vel;
  //std::vector<double> accel_covar; //What is the right type for ROS float64[9]?? boost::array< >?
  //std::vector<double> gyro_covar;
  //std::vector<double> mag_covar;
  //std::vector<double> quat_covar;

  float compass_heading;



 
 


  //parameters
  int rate_;
  double timeout_sec_;
  
};

#endif
