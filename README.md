# wheele
Wheele Navigation ROS robot
Place this folder in your catkin_ws/src directory.
All wheele packages will be subdirectories of this wheele folder (repository).
See https://answers.ros.org/question/257855/git-strategy-for-catkin-and-package-folders/

## Useful Reference Links:
http://henrysbench.capnfatz.com/henrys-bench/arduino-projects-tips-and-more/arduino-can-bus-module-1st-network-tutorial/

## Dependencies
### (external) ROS packages used:
NONE at the moment I believe. Possible future dependencies:
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
