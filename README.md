# wheele
Wheele Navigation ROS robot
Place this folder in your catkin_ws/src directory.
All wheele packages will be subdirectories of this wheele folder (repository).
See https://answers.ros.org/question/257855/git-strategy-for-catkin-and-package-folders/

## ROS Node Overview
### can2ros.py in can2ros_wheele
Subscribes to:  
`/imu` (sends base_link laser tf)  
`/mag_imu` (sets odom_heading initially, sends map odom tf)  
`/gps_pose` (updates odom_heading)

Reads CAN data using python CAN library, encoders, gyro (publishes odom)  
TODO: subscribe to `/received_msgs` from socketcan_bridge  
?? TODO: remove topics `/wheele_cmd_man`, `/auto_raw`, `/raw_cmd_py` ??

### avoid_obs (AvoidObs.cpp) in avoid_obstacles
See avoid_obstacles.launch  
Subscribes to `/odom` for scan in odom costmap  
Subscribes to `/scan`, see scanCallback, checks for cone obstacle, publishes `/costmap` (in odom frame)  
check_for_cone_obstacle() publishes `/obs_cone_pose` if cost cell is near estimated cone pose  
check_for_cone_obstacle() clears the cone obstacle cells from the costmap  
Subscribes to `/wp_cone_pose`, cone pose estimate published by waypoints_manager.py  
Currently, waypoints_manager.py passes `/raw_cone_pose` from camera straight thru

main loop runs update_plan() but this does nothing unless potential fields is used  
Only publishes `/cmd_vel`, `/pfObs` if potential fields is used  
`/move_base_simple/goal` goalCallback only relates to potential fields  

### astar (Astar.cpp) in avoid_obstacles
See avoid_obstacles.launch  
Subscribes to `/wp_goal` (in odom frame, from waypoints_manager.py)  
Subscribes to `/odom` so it knows where bot is in costmap  
Subscribes to `/costmap`, which calls get_path(), publishes `/path`  
get_path() can return false and not publish `/path` for multiple reasons:  
No solution found, goal is in obstacle cell

### nav_states (NavStates.cpp) in avoid_obstacles
A cleaner and improved implementation of waypoints_manager.py and wheele_local_planner.py  
See below. Note topic /found_cone is now obsolete.

### waypoints_manager.py in avoid_obstacles (obsolete, replaced by NavStates.cpp)
Publishes `/wp_goal` initially based on entered lat lon coordinates  
Subscribes to rviz 2d nav goal `/move_base_simple/goal`, publishes it on `/wp_goal`  
Subscribes to `/raw_cone_pose` from camera, publishes it on `/wp_cone_pose` which is used by avoid_obs to send back `/obs_cone_pose`  
Subscribes to `/obs_cone_pose` publishes it on `/wp_goal`, publishes `/found_cone`  
`/found_cone` will allow wheele to drive thru obstacles (it thinks they are the cone)  
TODO: rename wp_pub publisher to wp_goal to help us follow the data flow

### wheele_local_planner.py in ros_vehicle_model (obsolete, replaced by NavStates.cpp)
Subscribes to `/path`, publishes `/cmd_vel`  
Subscribes to `/found_cone`  
Subscribes to `/scan`, reverses when close to obstacles in front IF NOT `/found_cone`  
See execute_plan() for debugging when close cone stalking may override navigation logic  

### cone_finder.py in cone_findner
Subscribes to `/camera/image_raw`  
Publishes images `/cone_img`, `/hsv_filt` view with rqt_image_view  
Publishes pose `/raw_cone_pose` read by waypoints_manager.py  
See camera_test.launch in ros_vehicle_model/launch for camera setup.

### imu_stream (ImuCAN.cpp) in imu_can
Publishes `/imu`, `/mag_imu`, `/magXYZ`

### gps_transform.py in https://github.com/coderkarl/ros_misc/blob/master/gps_modular/scripts/gps_transform.py
Subscribes to `/fix`  
publishes `/odom_gps`, sends map base_link_gps tf
publishes `gps_pose`, (x,y) in meters in map frame w.r.t. initial lat, lon readings
TODO: verify the tf is between map andn base_link_gps  
Put gps_transform.py in a wheele package

### NMEA serial driver
nmea_serial_driver publishes gps data on topic `/fix`


## ros_vehicle_model package
Obsolete nodes: VehicleModel, MicroMaestro  
scripts/ nodes: wheele_local_planner.py  
src/ nodes: ros2can

### ros2can (ROS2CAN.cpp)
Subcribes to `/cmd_vel` (v.x m/s, w.z rad/sec), sends it over CAN id 0x301  
The cmd CAN output will be zero if topic cmd_vel is not received within param timeout_sec  
vel_mm_HB LB yaw_rate_decideg_HB LB 80 00 80 00 (in hexadecimal here, e.g. candump can0)  
Unit Test shows CAN data in decimal format using ROS:  
Note in ros topic `/sent_messages` -0.1 m/s, -0.3 rad/sec will be:  
data: [127, 156, 127, 85, 128, 0, 128, 0]
```
export ROS_MASTER_URI=http://<hostname>:11311/
# ~/.bashrc, likely your laptop hostname
# Uncommenting the wheelepi one should automatically set your laptop as ROS_MASTER_URI
roscore  
rosrun ros_vehicle_model ros2can  
rostopic echo /sent_messages | grep data #remove grep if multiple can IDs in future  
rostopic pub --once /cmd_vel geometry_msgs/Twist  '{linear:  {x: -0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: -0.3}}'
```

## Useful Reference Links:
http://henrysbench.capnfatz.com/henrys-bench/arduino-projects-tips-and-more/arduino-can-bus-module-1st-network-tutorial/

## Dependencies
### (external) ROS packages used:

`sudo apt-get install ros-kinetic-ros-canopen`

Possible future dependencies:
```
cd catkin_ws/src/
git clone https://github.com/scanse/sweep-ros.git`
git clone https://github.com/scanse/sweep-sdk
cd sweep-sdk/
cd libsweep/
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
sudo cmake --build . --target install
sudo ldconfig
```
`sudo apt-get install ros-kinetic-pointcloud-to-laserscan`
`sudo apt-get install ros-kinetic-navigation`
`sudo apt-get install ros-kinetic-amcl`

### Arduino libraries used:
(may need to move .h and .cpp files up one level to top level lib folder)

git clone https://github.com/Seeed-Studio/CAN_BUS_Shield.git (mcp_can for Nano)

https://github.com/sparkfun/SparkFun_CAN-Bus_Arduino_Library/archive/master.

https://github.com/sparkfun/CAN-Bus_Shield/tree/master/Libraries/Arduino/src (mcp2515 for Uno)

git clone https://github.com/GreyGnome/PinChangeInt.git

### python-can (python 2)
`pip install python-can`

### python-can (python 3)
```
wget "https://bitbucket.org/hardbyte/python-can/get/4085cffd2519.zip"
unzip 4085cffd2519.zip
mv hardbyte-python-can-4085cffd2519/ hardbyte-python-can
rm 4085cffd2519.zip
cd hardbyte-python-can/
sudo python3 setup.py install
```
### can-utils
`sudo apt-get install can-utils`

### CAN and SPI on rpi
modify /boot/config.txt
```
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
dtoverlay=spi-bcm2835
```
To use can in linux:
```
sudo ip link set can0 up type can bitrate 500000
candump can0
cansend can0 idh#dd.dd.dd.dd.dd.dd.dd.dd
