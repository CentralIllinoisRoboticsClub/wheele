cmake_minimum_required(VERSION 2.8.3)
project(imu_can)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  can_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
)

## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
INCLUDE_DIRS include
#  LIBRARIES imu_can
#  CATKIN_DEPENDS can_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include include/imu_can_lib)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(imu_can
#   src/${PROJECT_NAME}/ImuCAN.cpp
# )

## Declare a C++ executable
add_executable(imu_stream src/ImuCAN.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(imu_stream
  ${catkin_LIBRARIES}
)
