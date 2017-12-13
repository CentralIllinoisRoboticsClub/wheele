#ifndef VehicleModel_HPP
#define VehicleModel_HPP

//C++ Includes
//#include <iostream>
//#include <thread>
//#include <boost/thread.hpp> ???
//#include <chrono>

//OS Includes
//#include <unistd.h>

//ROS Includes
#include <ros/ros.h>
#include <wheele_msgs/SpeedCurve.h>
#include <wheele_msgs/ThrustSteer.h>
//#include <std_msgs/String.h>
//#include <geometry_msgs/Twist.h>

namespace CIRC
{
    class VehicleModel
    {
        public:
	    VehicleModel();
	    //~VehicleModel();
        private:
	    	void driveCallback(const wheele_msgs::SpeedCurve& spdCrv);
		//void driveCallback(const wheele_msgs::SpeedCurve::ConstPtr& spdCrv);
	    	
		void calculateSteerThrottle(
			float speed, float curvature,
			float *throttleLeft, float *throttleRight,
			float *steerLeft, float *steerRight);
	    	
		bool enableLogging = false;
		// Wheel-E geometry constants
		const float WHEELBASE_WIDTH = (23 * 2.54 / 100);		// Meters
		const float WHEELBASE_LENGTH = (12.5 * 2.54 / 100);	// Meters
		const float FULL_THROTTLE_SPEED = 16.0;	// Speed in m/s when throttle is 1.0
        const float MAX_CURVATURE = 2.0; // Max curvature 1/m

	  	ros::NodeHandle nh_; 
	  	ros::Publisher pub_;
	  	ros::Subscriber sub_;

		//parameters For future reference, see VehicleModel.cpp constructor
		//int raw_cmd_max_;
		//double linear_scale_, botwidth_;
    };
}

#endif
