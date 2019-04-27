# wheele
Wheele Navigation ROS robot
Place this folder in your catkin_ws/src directory.
All wheele packages will be subdirectories of this wheele folder (repository).
See https://answers.ros.org/question/257855/git-strategy-for-catkin-and-package-folders/

## ros_vehicle_model package
Obsolete nodes: VehicleModel, MicroMaestro
scripts/ nodes: wheele_local_planner.py
src/ nodes: ros2can

### ros2can (ROS2CAN.cpp)
Subcribes to /cmd_vel (v.x m/s, w.z rad/sec), sends it over CAN id 0x301  
vel_mm_HB LB yaw_rate_decideg_HB LB 80 00 80 00 (in hexadecimal here, e.g. candump can0)
Unit Test shows CAN data in decimal format using ROS:  
Note in ros topic /sent_messages -0.1 m/s, -0.3 rad/sec will be:  
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
