// Copyright 2019 coderkarl. Subject to the BSD license.

#include "ros_vehicle_model/ROS2CAN.h"
#include <math.h>

#define CMD_VEL_CAN_ID 0x301

/**********************************************************************
* Wheele Specific ROS to CAN communication
* Requires:
* sudo apt install ros-kinetic-ros-opencan
* rosrun socketcan_bridge socketcan_bridge_node
*   The socket_can bridge turns CAN into /received_messages topic
*   The socket_can bridge turns /sent_messages into CAN 
* 
* Subscribe to /cmd_vel, Publish it on /sent_messages CAN id 0x301
**********************************************************************/

//Constructor
ROS2CAN::ROS2CAN()
{
    //Topics you want to publish
    canFrame_pub_ = nh_.advertise<can_msgs::Frame>("sent_messages", 5); //send CAN

    //Topic you want to subscribe
    cmd_sub_ = nh_.subscribe("cmd_vel", 5, &ROS2CAN::cmdVelCallback, this); //receive cmd_vel of type geometry_msgs::Twist
    
    nh_p  = ros::NodeHandle("~");
    nh_p.param("rate_hz", rate_, 10); // cmd_vel will be sent out on CAN at this rate
    nh_p.param("timeout_sec", timeout_sec_,1.0); //cmd_vel to CAN will be zero if no cmd_vel is received after timeout_sec
    ROS_INFO("rate_hz = %d, timeout_sec = %0.1f", rate_, timeout_sec_);
    
    //Constant properties of published data
    canFrame.is_extended = false;
    canFrame.dlc = 8; //Number of bytes sent
}

//Default destructor.
ROS2CAN::~ROS2CAN() {}

int ROS2CAN::get_rate()
{
    return rate_;
}

void ROS2CAN::cmdVelCallback(const geometry_msgs::Twist& cmd)
{    
  latest_cmd_vel = cmd;
  cmd_vel_time = ros::Time::now();
  // sendCmdVel(); // If you want to have cmd_vel from the planner control the rate
}

void ROS2CAN::sendCmdVel()
{
  double vel = latest_cmd_vel.linear.x;
  double yaw_rate_deg = latest_cmd_vel.angular.z * 180.0/M_PI;
  if( (ros::Time::now() - cmd_vel_time).toSec() > timeout_sec_)
  {
    vel = 0;
    yaw_rate_deg = 0;
  }
  int16_t vel_mm = int(vel*1000);
  int16_t yaw_rate_decideg = int(yaw_rate_deg*10);
  tx_can(CMD_VEL_CAN_ID, vel_mm, yaw_rate_decideg, 0, 0);
}

void ROS2CAN::tx_can(uint16_t id, int16_t num1, int16_t num2, int16_t num3, int16_t num4)
{
  uint16_t raw1 = num1 + 32768;
  uint16_t raw2 = num2 + 32768;
  uint16_t raw3 = num3 + 32768;
  uint16_t raw4 = num4 + 32768;

  canFrame.id = id;
  canFrame.header.stamp = ros::Time::now();

  canFrame.data[0] = getByte(raw1,1);
  canFrame.data[1] = getByte(raw1,0);
  canFrame.data[2] = getByte(raw2,1);
  canFrame.data[3] = getByte(raw2,0);
  canFrame.data[4] = getByte(raw3,1);
  canFrame.data[5] = getByte(raw3,0);
  canFrame.data[6] = getByte(raw4,1);
  canFrame.data[7] = getByte(raw4,0);

  canFrame_pub_.publish(canFrame);
}

uint8_t ROS2CAN::getByte(uint16_t x, unsigned int n)
{
  return (x >> 8*n) & 0xFF;
}

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "ros2can");

    ROS2CAN ros_to_can;
    ROS_INFO("Starting Wheele ROS to CAN");
    int loop_time = ros_to_can.get_rate();
    ros::Rate rate(loop_time);

    while(ros::ok())
    {
        ros::spinOnce();
        ros_to_can.sendCmdVel();
        rate.sleep();
    }

    return 0;
}
