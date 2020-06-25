#include "ros_micromaestro/MicroMaestro.hpp"

using namespace std;
using namespace CIRC;

//Default constructor.
MicroMaestro::MicroMaestro()
{
    //Topic you want to publish, NOTHING NEEDED FOR NOW
    //pub_ = nhg_.advertise<wheele_msgs::MSG_TYPE>("future_topic_from_MM", 1);

    //Topic you want to subscribe
    sub_ = nhg_.subscribe("thrust_and_steer", 1, &MicroMaestro::cmdMicMasCallback, this);
}

//Default destructor.
MicroMaestro::~MicroMaestro()
{
	ROS_INFO("Shutting down...");
	if (deviceIsOpen)
    {
        ROS_INFO("Closing serial device.");
		delete serialInterface;
		serialInterface = NULL; // Need this?
    }
	else
	{
		ROS_INFO("Serial device was not open");
	}
}

bool MicroMaestro::initialize()
{
    ros::NodeHandle nh("~");

    bool success = true;

    // Load parameters
    //ROS_INFO("Loading pololu_motors_yaml...");
    // Load named motors
	/*
    if (nh.hasParam("pololu_motors_yaml"))
    {
        nh.getParam("pololu_motors_yaml", pololu_config_dir);
        PololuYamlParser::parse(pololu_config_dir, motors);
    }*/

	// Install the symlink file in the symlink-mm folder to use the /dev/MicroMaestro alias
    nh.param<string>("device_name", deviceName, "/dev/MicroMaestro");
    nh.param<int>("baud_rate", baudRate, 115200);
    nh.param<int>("rate_hz", rate_hz, 10);
    //nh.param<bool>("daisy_chain", daisy_chain, false);
    //nh.param<string>("topic_prefix", topic_prefix, "pololu/");
    //nh.param<string>("topic_name", topic_name, "command");

    // Populate servo objects
	servoInit();

    //Attempt to create serial device.
	try {
		serialInterface = RPM::SerialInterface::createSerialInterface( deviceName, baudRate , &errorMessage );
		if ( !serialInterface )
		{	
			// Log Error from the RPM library
			ROS_ERROR("Failed to create serial interface:");
			cout <<"Failed to create serial interface\n\r";
			//ROS_ERROR(errorMessage.c_str()); //where is errorMesssage declared? MicMas header
			success = false;
		} // Need to dig into whether this connection is blocking or nonblocking
		else
		{
			deviceIsOpen = true;
			cout <<"Successfuly opened serial device.\n\r";
			ROS_INFO("Successfully opened serial device.");
			this_thread::sleep_for(chrono::seconds(1)); //Should I use a sleep from ros.h?
		}
	}
    catch (exception& e) // Do I need more to use thie exception& e ?? Header? Declaration?
    {
        ostringstream oss;
        oss << "Strange error occurred opening serial port. The error was: ";
        oss << e.what();
        //ROS_ERROR(oss.str());
		ROS_ERROR("Strange error opening serial port.");
    }

    return success;
}


double MicroMaestro::get_rate_hz() //may need to be a double because it is passed to ros::Rate rate() in main
{
    return rate_hz;
}

void MicroMaestro::cmdMicMasCallback(const wheele_msgs::ThrustSteer& drive_cmd)
{
	float throttleLeft, throttleRight;	// From vehicle_model topic "thrust_and_steer"
	float steerLeft, steerRight;		// From vehicle_model topic "thrust_and_steer"
	
	steerLeft = (drive_cmd.steerLeft)*radToDeg;
	cout << "Steer left = " << steerLeft << "\n\r";
    Servo_LF.setCommand(
		clampPulse(
			convertToPulse(
				-steerLeft, 
				(float)Servo_LF.getMin(), 
				(float)Servo_LF.getCenter(), 
				(float)Servo_LF.getMax(), 
				(float)maxPosDeg, 
				(float)minNegDeg
			), 
			serialInterface->getMaxChannelValue(), serialInterface->getMinChannelValue()
		)
	);
    Servo_LR.setCommand( clampPulse(
		convertToPulse(
			steerLeft, 
			(float)Servo_LR.getMin(), 
			(float)Servo_LR.getCenter(), 
			(float)Servo_LR.getMax(), 
			(float)maxPosDeg, 
			(float)minNegDeg ),
		serialInterface->getMaxChannelValue(), serialInterface->getMinChannelValue()) );
    serialInterface->setTargetCP( Servo_LF.getChannel(), Servo_LF.getCommand() );
    serialInterface->setTargetCP( Servo_LR.getChannel(), Servo_LR.getCommand() );
	cout << "Left front steer pulse = " << Servo_LF.getCommand()/4 << "\n\r";
	cout << "Left rear steer pulse = " << Servo_LR.getCommand()/4 << "\n\r";


	steerRight = (drive_cmd.steerRight)*radToDeg;
	cout << "Steer right = " << steerRight << "\n\r";
    Servo_RF.setCommand( clampPulse(
		convertToPulse(
			-steerRight, 
			(float)Servo_RF.getMin(), 
			(float)Servo_RF.getCenter(), 
			(float)Servo_RF.getMax(), 
			(float)maxPosDeg, 
			(float)minNegDeg ),
		serialInterface->getMaxChannelValue(), serialInterface->getMinChannelValue()) );
    Servo_RR.setCommand( clampPulse(
		convertToPulse(
			steerRight, 
			(float)Servo_RR.getMin(), 
			(float)Servo_RR.getCenter(), 
			(float)Servo_RR.getMax(), 
			(float)maxPosDeg, 
			(float)minNegDeg ),
		serialInterface->getMaxChannelValue(), serialInterface->getMinChannelValue()) );
	serialInterface->setTargetCP( Servo_RF.getChannel(), Servo_RF.getCommand() );
	serialInterface->setTargetCP( Servo_RR.getChannel(), Servo_RR.getCommand() );
	cout << "Right front steer pulse = " << Servo_RF.getCommand()/4 << "\n\r";
	cout << "Right rear steer pulse = " << Servo_RR.getCommand()/4 << "\n\r";



	throttleLeft = drive_cmd.throttleLeft;
	cout << "Throttle left = " << throttleLeft << "\n\r";
	ESC_L.setCommand( clampPulse(
		convertToPulse(
			throttleLeft, 
			(float)ESC_L.getMin(), 
			(float)ESC_L.getCenter(), 
			(float)ESC_L.getMax(), 
			(float)maxThrottle, 
			(float)minThrottle ),
		serialInterface->getMaxChannelValue(), serialInterface->getMinChannelValue()) );
    serialInterface->setTargetCP( ESC_L.getChannel(), ESC_L.getCommand() );
	cout << "Left throttle pulse = " << ESC_L.getCommand()/4 << "\n\r";


	throttleRight = drive_cmd.throttleRight;
	cout << "Throttle right = " << throttleRight << "\n\r";
	ESC_R.setCommand( clampPulse(
		convertToPulse(
			throttleRight, 
			(float)ESC_R.getMin(), 
			(float)ESC_R.getCenter(), 
			(float)ESC_R.getMax(), 
			(float)maxThrottle, 
			(float)minThrottle ),
		serialInterface->getMaxChannelValue(), serialInterface->getMinChannelValue()) );
    serialInterface->setTargetCP( ESC_R.getChannel(), ESC_R.getCommand() );
	cout << "Right throttle pulse = " << ESC_R.getCommand()/4 << "\n\r";
}

/*void MicroMaestro::errorStateEvent()
{
    //Continue attempting to read data but leave
    //an active fault until data are read.
    try
    {
        readWriteUSB();
        ROS_ERROR("In error.");
    }
    catch ()
    {
	}
}*/

void MicroMaestro::servoInit()
{
	// Init RF Servo
	Servo_RF.setChannel(CIRC::Servo::MMChannel::STEER_RF);
	Servo_RF.setPWM(pulseDataRF[0],pulseDataRF[1],pulseDataRF[2]);

	// Init RR Servo
	Servo_RR.setChannel(CIRC::Servo::MMChannel::STEER_RR);
	Servo_RR.setPWM(pulseDataRR[0],pulseDataRR[1],pulseDataRR[2]);

	// Init LF Servo
	Servo_LF.setChannel(CIRC::Servo::MMChannel::STEER_LF);
	Servo_LF.setPWM(pulseDataLF[0],pulseDataLF[1],pulseDataLF[2]);

	// Init LR Servo
	Servo_LR.setChannel(CIRC::Servo::MMChannel::STEER_LR);
	Servo_LR.setPWM(pulseDataLR[0],pulseDataLR[1],pulseDataLR[2]);

	// Init ESC L
	ESC_L.setChannel(CIRC::Servo::MMChannel::ESC_L);
	ESC_L.setPWM(pulseDataESC[0],pulseDataESC[1],pulseDataESC[2]);

	// Init ESC L
	ESC_R.setChannel(CIRC::Servo::MMChannel::ESC_R);
	ESC_R.setPWM(pulseDataESC[0],pulseDataESC[1],pulseDataESC[2]);
}

// Convert throttle or steering inputs to pulse commands
unsigned short MicroMaestro::convertToPulse(float Data, float Min, float Center, float Max, float Max_Pos, float Min_Neg)
{

	// Define variables for calculation
	float Pct, Output;

	// If input steering / throttle commands are positive
	if(Data >= 0){
		// Calculate scaling percentage
		Pct = Data / Max_Pos;
		// Calculate output
		Output = Pct * (Max - Center) + Center; }
	// If steering or throttle commands are negative
	else {
		Pct = Data / Min_Neg;
		Output = Center - Pct * (Center - Min);	}

	// Convert to quarter microseconds
	Output = Output * 4; 
	// Return output
	return (unsigned short)Output;

}

// Clamp data between min and max acceptable range
// Python / ROS would clamp steering commands but that's being taken care of in the vehicle model
// Use this to clamp the pulse commands going to the micromaestro
unsigned short MicroMaestro::clampPulse(unsigned short Data, unsigned short MaxData, unsigned short MinData)
{
	return max(min(MaxData, Data), MinData);
}
