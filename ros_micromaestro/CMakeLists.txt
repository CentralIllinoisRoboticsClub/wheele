cmake_minimum_required(VERSION 2.8.3)
project(ros_micromaestro)

##-std=c++11
if(CMAKE_COMPILER_IS_GNUCXX)
add_definitions(-std=gnu++0x)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  wheele_msgs
)

## Generate added messages and services with any dependencies listed here
# generate_messages( ##May not need
#  DEPENDENCIES
#  std_msgs
#  wheele_msgs
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#LIBRARIES ros_micromaestro
#CATKIN_DEPENDS roscpp std_msgs wheele_msgs #NEED ALL THIS KARL? 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
## IF I MOVE MicroMaestro.hpp TO SIMPLY /include AND COMMENT THE NEXT LINE, BUILD CANNOT FIND THE .hpp
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
#add_library(ros_micromaestro
#  src/${PROJECT_NAME}/MicroMaestro.cpp
#)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(ros_vehicle_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(micromaestro
 src/MicroMaestro.cpp
 src/main.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(ros_vehicle_model_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## NOW THAT I CAN BUILD ONE PACKAGE AT A TIME, I DON'T KNOW IF I NEED THIS TO GET wheele_msgs ready
#add_dependencies(micromaestro micromaestro_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_library(lib1 src/Servo.cpp)
add_library(lib2 src/RPMSerialInterface.cpp src/RPMSerialInterfacePOSIX.cpp)
#add_library(include/micromaestro_lib/Servo.hpp include/micromaestro_lib/RPMSerialInterface.h)
## Specify libraries to link a library or executable target against
target_link_libraries(micromaestro lib1 lib2
  ${catkin_LIBRARIES}
)
