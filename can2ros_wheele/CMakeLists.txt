cmake_minimum_required(VERSION 2.8.3)
project(can2ros_wheele)

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
  wheele_msgs
)

catkin_package(
INCLUDE_DIRS include
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
# add_library(can2ros
#   src/${PROJECT_NAME}/Can2Ros.cpp
# )

## Declare a C++ executable
add_executable(can2ros src/Can2Ros.cpp)
add_dependencies(can2ros ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(can2ros
  ${catkin_LIBRARIES}
)
