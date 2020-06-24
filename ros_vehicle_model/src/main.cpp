#include "ros_vehicle_model/VehicleModel.hpp"
//#include <iostream>
//#include <unistd.h>

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "vehicle_model");

  //Create an object of VehicleModel that will take care of everything
  CIRC::VehicleModel vm;

  ros::spin();

  return 0;
}
