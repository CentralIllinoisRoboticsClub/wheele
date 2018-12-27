cmake_minimum_required(VERSION 2.8.3)
project(cone_finder)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  geometry_msgs
  sensor_msgs
  std_msgs
  tf_conversions
  image_transport
  dynamic_reconfigure
  tf
)

find_package(OpenCV REQUIRED)

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/Cone.cfg
  #...
)
