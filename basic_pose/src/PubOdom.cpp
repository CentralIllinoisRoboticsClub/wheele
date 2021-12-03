// Copyright 2019 coderkarl. Subject to the BSD license.

#include "../include/basic_pose/PubOdom.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include "geometry_msgs/Quaternion.h"

PubOdom::PubOdom() :
m_curTime(0),
m_prevTime(0),
m_encInitialized(false),
m_timeInitialized(false),
m_gyro_z_rad(0.0),
m_left_enc(0),
m_prev_left_enc(0),
m_right_enc(0),
m_prev_right_enc(0),
m_botx(0.0),
m_boty(0.0),
m_yaw_deg(0.0),
m_dist_sum(0.0),
m_time_sum(0.0),
m_velx(0.0)
{
  //Topics you want to publish
  m_odom_pub = m_nh.advertise<nav_msgs::Odometry>("odom", 50);

  m_enc_sub = m_nh.subscribe("encoders", 50, &PubOdom::encoderCallback, this);

  m_curTime = ros::Time::now();
  m_prevTime = m_curTime;
}

PubOdom::~PubOdom()
{

}

void PubOdom::imuCallback(const sensor_msgs::Imu &imu)
{
  m_gyro_z_rad = imu.angular_velocity.z; // rad/sec
  if(m_encInitialized)
  {
    updateOdom();
  }
}

void PubOdom::encoderCallback(const wheele_msgs::Encoder &data)
{
  m_left_enc = data.left_enc;
  m_right_enc = data.right_enc;

  if(!m_encInitialized)
  {
    m_encInitialized = true;
    m_prev_left_enc = m_left_enc;
    m_prev_right_enc = m_right_enc;
  }
}

void PubOdom::updateOdom()
{
  m_curTime = ros::Time::now();
  double dt = (m_curTime - m_prevTime).toSec();

  double gyro_thresh_dps = 0.00;
  double g_bias_dps = 0.0;
  double MAX_DTHETA_GYRO_deg = 100;
  double BOT_WIDTH = (23.0 * 2.54 / 100.0); //meters
  double COUNTS_PER_METER = 900.0;

  double gyroz_raw_dps = m_gyro_z_rad * 180.0 / M_PI;

  int16_t delta_left_enc = m_left_enc - m_prev_left_enc;
  int16_t delta_right_enc = m_right_enc - m_prev_right_enc;

  double dtheta_enc_deg = double(delta_right_enc - delta_left_enc) / COUNTS_PER_METER / BOT_WIDTH * 180.0 / 3.1416;

  double dmeters = double(delta_left_enc + delta_right_enc)/2.0 / COUNTS_PER_METER;
  double gz_dps = 0.0;
  double dtheta_gyro_deg = 0.0;

  if(abs(gyroz_raw_dps+g_bias_dps) < gyro_thresh_dps)
  {
    gz_dps = 0.0;
    dtheta_gyro_deg = 0.0;
  }
  else
  {
    gz_dps = gyroz_raw_dps+g_bias_dps;
    dtheta_gyro_deg = gz_dps*dt*360.0/368.2; //*360.0/375.0 //Scaling needed due to static pitch/roll IMU mount?
  }

  double dtheta_deg = dtheta_gyro_deg;
  if(fabs(dtheta_gyro_deg) > MAX_DTHETA_GYRO_deg)
  {
    //print('no gyro')
    dtheta_deg = dtheta_enc_deg;
  }

  //update bot position
  //self.bot.move(dmeters,dtheta_deg,use_gyro_flag)
  m_yaw_deg = m_yaw_deg + dtheta_deg;
  double dx = dmeters*cos(m_yaw_deg*3.1416/180);
  double dy = dmeters*sin(m_yaw_deg*3.1416/180);
  m_botx = m_botx + dx;
  m_boty = m_boty + dy;

  // update bot linear x velocity every 150 msec
  // need to use an np array, then push and pop, moving average
  m_dist_sum = m_dist_sum + dmeters;
  m_time_sum = m_time_sum + dt;
  if(m_time_sum > 0.15)
  {
    m_velx = m_dist_sum / m_time_sum;
    m_dist_sum = 0.0;
    m_time_sum = 0.0;
  }

  publishOdom();

  m_prevTime = m_curTime;
  m_prev_left_enc = m_left_enc;
  m_prev_right_enc = m_right_enc;
}

void PubOdom::publishOdom()
{
  tf2::Quaternion odom_quat;
  odom_quat.setRPY(0, 0, m_yaw_deg * M_PI / 180.0);

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = m_curTime;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = m_botx;
  odom_trans.transform.translation.y = m_boty;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation.x = odom_quat.x();
  odom_trans.transform.rotation.y = odom_quat.y();
  odom_trans.transform.rotation.z = odom_quat.z();
  odom_trans.transform.rotation.w = odom_quat.w();
  //odom_trans.transform.rotation = tf2::toMsg<tf2::Quaternion, geometry_msgs::Quaternion>(odom_quat);

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = m_curTime;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = m_botx;
  odom.pose.pose.position.y = m_boty;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = odom_quat.x();
  odom.pose.pose.orientation.y = odom_quat.y();
  odom.pose.pose.orientation.z = odom_quat.z();
  odom.pose.pose.orientation.w = odom_quat.w();
  //odom.pose.pose.orientation = tf2::toMsg<tf2::Quaternion, geometry_msgs::Quaternion>(odom_quat);

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = m_velx;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = m_gyro_z_rad;

  //publish the message
  m_odom_pub.publish(odom);
}

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "pubOdom");

  PubOdom pub_odom;
  ROS_INFO("Starting PubOdom");
  int loop_rate = 50;
  ros::Rate rate(loop_rate);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
