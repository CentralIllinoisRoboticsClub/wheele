#ifndef MICROMAESTRO_HPP
#define MICROMAESTRO_HPP

//C++ Includes
#include <iostream> //std::cout, std::endl
#include <thread> //std::this_thread::sleep_for
#include <chrono> //std::chrono::seconds
#include <sstream>
//#include <algorithm>
//#include <sstream> //Included twice?

//OS Includes
//#include <unistd.h>
#include <math.h>

// MicroMaestro Serial Interface Include
#include "RPMSerialInterface.h"
// Servo class object is in its own header file
#include "Servo.hpp"

//ROS Includes
#include <ros/ros.h>
#include <wheele_msgs/ThrustSteer.h>

namespace CIRC
{
	
    class MicroMaestro
	{
		public:
		    MicroMaestro();
		    ~MicroMaestro();
			bool initialize();
			double get_rate_hz();
		private:
			ros::NodeHandle nhg_; 
		  	//ros::Publisher pub_;
		  	ros::Subscriber sub_;
			void cmdMicMasCallback(const wheele_msgs::ThrustSteer& drive_cmd);

		    //void errorStateEvent() override;
			
		    //void readWriteUSB();
			unsigned short convertToPulse(float Data, float Min, float Center, float Max, float Max_Pos, float Min_Neg);
			unsigned short clampPulse(unsigned short Data, unsigned short MaxData, unsigned short MinData);
			void servoInit();
		    
			//polysync::datamodel::SensorDescriptor sd; // what was the purpose of sd, ROS equivalent?
			//ps_timestamp lastReadTime; // what was the purpose of lastReadTime, ROS equivalent?

			// Replaced this with int rate_hz like in PololuController.h
			//const std::chrono::milliseconds loopTime = std::chrono::milliseconds(100);

			
			bool enableLogging = false;

			// Install the symlink file in the symlink-mm folder to use the /dev/MicroMaestro alias
			int baudRate, rate_hz; //Can ROS parameters be an unsigned int?
			std::string deviceName;
			std::string errorMessage;
			RPM::SerialInterface* serialInterface;
			bool deviceIsOpen;
			long readErrorCount;
			const unsigned short int acceptableReadErrors = 10;
			
			
			// Max and min degrees of steer servos
			const int maxPosDeg =  45;
			const int minNegDeg = -45;

			// Conversion factor of rad to degrees
			const float radToDeg = 180/M_PI;
			
			// Throttle Min/Max
			const int minThrottle = -1; // Full Reverse
			const int maxThrottle =  1; // Full Forward

			// Min Center and Max commands to be sent to servos.  Adjust/Tune where necessary
			const unsigned short pulseDataRF[3] = {1030, 1500, 1840};		// Right-Front
			const unsigned short pulseDataRR[3] = {1300, 1620, 2000};		// Right-Rear
			const unsigned short pulseDataLF[3] = {1250, 1600, 2000};		// Left-Front
			const unsigned short pulseDataLR[3] = {1150, 1540, 1900};		// Left-Rear
			const unsigned short pulseDataESC[3] = {1000, 1500, 2000};		// Left/Right ESC
			
			// Initialize servo objects
			Servo Servo_RF, Servo_RR, Servo_LF, Servo_LR, ESC_L, ESC_R;	
	};
}
#endif
