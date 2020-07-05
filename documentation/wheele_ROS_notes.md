## Wheele ROS Notes
coderkarl

# 1  Wheele Messages package

package: wheele_msgs
msg/	ThrustSteer.msg	SpeedCurve.msg

rostopic pub -1 wheele_cmd_vel   wheele_msgs/SpeedCurve   - -    '8.0'   '0.0'

rostopic pub -1 wheele_cmd_vel   wheele_msgs/ThrustSteer   - -  
		'throttleLeft'   'throttleRight' ‘steerLeft’ 	‘steerRight’

# 2  Vehicle Model package

package: ros_vehicle_model
src/	VehicleModel.cpp	main.cpp
include/vehicle_model_lib/	VehicleModel.hpp


Vehicle Model subscribes to:
topic “wheele_cmd_vel”
type /wheele_msgs/SpeedCurve
rostopic pub -1 wheele_cmd_vel   wheele_msgs/SpeedCurve   - -    '8.0'   '0.0'


Vehicle Model publishes:
topic “thrust_and_steer”
type /wheele_msgs/ThrustSteer
rostopic pub -1 wheele_cmd_vel   wheele_msgs/ThrustSteer   - -  
		'throttleLeft'   'throttleRight' ‘steerLeft’ 	‘steerRight’

Potential Build Error:
If catkin fails because the wheele_msgs/ThrustSteer.h file does not exist
First try catkin_make -j1
Then add the add_dependencies line after add_executable in CmakeLists.txt

VehicleModel, CmakeLists.txt
## Declare a C++ executable
add_executable(vehicle_model
 src/VehicleModel.cpp
 src/main.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(ros_vehicle_model_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(vehicle_model ros_vehicle_model_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(vehicle_model
  ${catkin_LIBRARIES}
)

# 3  Micro Maestro Board Setup

https://www.pololu.com/file/0J315/maestro-linux-150116.tar.gz
Extract and follow README.txt
sudo apt-get install libusb-1.0.0-dev mono-runtime
sudo apt-get install mono-reference-assemblies-2.0 mono-devel
(In place of ) sudo apt-get install libmono-winforms2.0-cil

Final SYMLINK rules used (Both CIRC rules and above rules did not work):
https://forum.pololu.com/t/distinguishing-multiple-maestros-in-linux/5715/5
You can also follow the same route using symlink-mm/ in the ros_micromaestro package folders.
I believe we need different rules because ros_pololu_servo requires the Micro Maestro to be dual port
I think we can remove the second KERNEL== line with the pololu_servo_ttl

SUBSYSTEM=="usb", ATTRS{idVendor}=="1ffb", MODE="0666"

KERNEL=="ttyACM*", SUBSYSTEMS=="usb", ACTION=="add", ATTRS{idVendor}=="1ffb", ATTRS{idProduct}=="008[0-f]", MODE="0666", PROGRAM="/bin/bash -c '/bin/echo %p | /bin/grep -c :1.0/tty'", RESULT=="1", SYMLINK+="MicroMaestro", GROUP="dialout"

KERNEL=="ttyACM*", SUBSYSTEMS=="usb", ACTION=="add", ATTRS{idVendor}=="1ffb", ATTRS{idProduct}=="008[0-f]", MODE="0666", PROGRAM="/bin/bash -c '/bin/echo %p | /bin/grep -c :1.2/tty'", RESULT=="1", SYMLINK+="polulu_servo_ttl_$attr{serial}", GROUP="dialout"

# 4  ros_pololu_servo package
WE MAY NOT NEED THIS IF WE CAN REUSE THE Servo and RPMSerialInterface C++ code.
What pros/cons do you see of this package vs. a custom one?
We may just want to experience making another custom ROS package for the micro maestro that is better than this package.

Could not get https://github.com/hansonrobotics/ros_pololu to work. Needed to properly install python dependency pololu-motors in ROS
https://github.com/hansonrobotics/pololu-motors
GAVE UP and found a different package that worked:

https://github.com/geni-lab/ros_pololu_servo
When I tried catkin_make, I got an error that ros_pololu_servo/MotorCommand.h does not exist
This should be created based on the MotorCommand.msg in the msg folder
http://answers.ros.org/question/188982/message-headers-wont-build-first/
http://answers.ros.org/question/73048/first-compile-with-messages-messages-not-found/

In CmakeLists.txt, I tried the following after the add_executable:
add_dependencies(ros_pololu_servo_node ros_pololu_servo_generate_messages_cpp)
Still failed
catkin_make -j1 worked
I also then ran catkin_make, not knowing if catkin_make -j1 was complete
Note, the add_dependencies can probably be removed and did not serve a purpose

ros_pololu_servo build (catkin_make) error revisited….
I needed add_dependencies for the library as well in CmakeLists.txt
add_library(ros_pololu_servo
   src/ros_pololu_servo/PololuMath.cpp
   src/ros_pololu_servo/PololuYamlParser.cpp
   src/ros_pololu_servo/PololuController.cpp
 )
add_dependencies(ros_pololu_servo ros_pololu_servo_generate_messages_cpp)

Go to maestro-linux folder, run ./MaestroControlCenter
Follow instructions from ros_pololu_servo README
In catkin_ws/src, modify ros_pololu_servo/launch/pololu_motors.yaml and pololu_example.launch

roscore
roslaunch ros_pololu_servo pololu_example.launch
rostopic pub -1 /pololu/command ros_pololu_servo/MotorCommand -- 'servo5' '-3.1' '0' '0'

I cannot understand the servo limits in the pololu_motors.yaml file
motor_id corresponds to the servo number on the micro maestro board

# 5  Micro Maestro package

package: ros_micromaestro
src/	MicroMaestro.cpp	main.cpp
include/vehicle_model_lib/	MicroMaestro.hpp


Micro Maestro subscribes to:
topic “thrust_and_steer”
type /wheele_msgs/ThrustSteer
rostopic pub -1 wheele_cmd_vel   wheele_msgs/ThrustSteer   - -  
		'throttleLeft'   'throttleRight' ‘steerLeft’ 	‘steerRight’


Micro Maestro publishes:
NOTHING (Current Decision 3/25/2017, Kirsch)
or
topic /pololu/command
type /ros_pololu_servo/MotorCommand
rostopic pub -1 /pololu/command ros_pololu_servo/MotorCommand -- 'servo5' '-3.1' '0' '0'

# ISSUES TO DISCUSS:

ISSUE 1 MicroMaestro
In MicroMaestro.hpp we have:
// Min Center and Max commands to be sent to servos.  Adjust/Tune where necessary
			const unsigned short pulseDataRF[3] = {1030, 1500, 1840};		// Right-Front
			const unsigned short pulseDataRR[3] = {1300, 1620, 2100};		// Right-Rear
			const unsigned short pulseDataLF[3] = {1250, 1600, 2020};		// Left-Front
			const unsigned short pulseDataLR[3] = {1150, 1540, 1900};		// Left-Rear
			const unsigned short pulseDataESC[3] = {1000, 1500, 2000};		// Left/Right ESC
			
These are then used in MicroMaestro::servoInit(), and the effects are used in
MicroMaestro::cmdMicMasCallback
Servo_RR.setCommand( 
		convertToPulse(
			-steerRight, 
			(float)Servo_RR.getMin(), 
			(float)Servo_RR.getCenter(), 
			(float)Servo_RR.getMax(), 
			(float)maxPosDeg, 
			(float)minNegDeg ) );
	serialInterface->setTargetCP( Servo_RF.getChannel(), Servo_RF.getCommand() );
	serialInterface->setTargetCP( Servo_RR.getChannel(), Servo_RR.getCommand() );
	cout << "Right front steer pulse = " << Servo_RF.getCommand()/4 << "\n\r";
	cout << "Right rear steer pulse = " << Servo_RR.getCommand()/4 << "\n\r";

If we look at serialInterface→setTargetCP, we will find that NOTHING is done if the pulse is outside of the hardcoded bounds defined in RPMSerialInterface.h
private:
	static const unsigned short mMinChannelValue = 3968;
	static const unsigned short mMaxChannelValue = 8000;

The hardcoded mMaxChannelValue in RPMSerialInterface.h does not agree with the bounds defined for the Right Rear and Left Front steering servos. So if we send a throttle-steer command that results in a Right Rear pulse of 2050, NOTHING happens to the servo. The pulse command must be less than 2000. (This is because the commands are multiplied by 4 before the last step to the Micro Maestro)

SHOULD WE INCREASE TO 8000 LIMIT TO 8400 (2100*4) ??

I noitced that MicroMaestro::clampPulse is never used and COULD solve this issue.
However, it may be better to modify RPMSerialInterface to do the clamping
getMinChannelValue()
Vehicle Model does NOT take care of the issue by limiting curvature or calculated steer angles.
Perhaps whatever sends curvature ensures we never have to worry about the issue.

ISSUE 2 MicroMaestro
^C     Segmentation fault (core dumped)
