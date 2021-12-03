// Copyright 2019 coderkarl. Subject to the BSD license.

#ifndef PubOdom_H
#define PubOdom_H

//ROS Includes
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Vector3.h"
#include "wheele_msgs/Encoder.h"
#include "nav_msgs/Odometry.h"
#include <vector>
#include <tf2_ros/transform_broadcaster.h>

class PubOdom
{
public:
  PubOdom();
  ~PubOdom();

private:
  void imuCallback(const sensor_msgs::Imu &imu);
  void encoderCallback(const wheele_msgs::Encoder &data);
  void updateOdom();
  void publishOdom();

  ros::NodeHandle m_nh;
  ros::Publisher m_odom_pub;
  ros::Subscriber m_imu_sub;
  ros::Subscriber m_enc_sub;

  tf2_ros::TransformBroadcaster odom_broadcaster;

  ros::Time m_curTime, m_prevTime;
  bool m_encInitialized;
  bool m_timeInitialized;

  double m_gyro_z_rad;
  int16_t m_left_enc, m_prev_left_enc;
  int16_t m_right_enc, m_prev_right_enc;

  double m_botx, m_boty, m_yaw_deg;
  double m_dist_sum;
  double m_time_sum;

  double m_velx;

};

#endif
