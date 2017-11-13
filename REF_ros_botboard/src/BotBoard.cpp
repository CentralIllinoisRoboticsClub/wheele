#include "BotBoard.hpp"

using namespace std;
//using namespace polysync;
using namespace CIRC;
//namespace dm = polysync::datamodel;

//Default constructor, with initialization list.
BotBoard::BotBoard() :
    outputTicks(false),
    deviceIsOpen(false),
    readErrorCount(0)
{
}

//Default destructor, see commented releaseStateEvent()
// Need to understand
    // RPM::SerialInterface* serialInterface; vs.
    // std::shared_ptr<polysync::Serial> serialDevice;
BotBoard::~BotBoard()
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

/*
void BotBoard::setConfigurationEvent(int argc, char * argv[])
{
                //cout << "    -e                Enable outputting encoder ticks to the command line." << endl;
                
            //case 'e':
            //    outputTicks = true;
}*/


bool BotBoard::initialize() // See commented initStateEvent() below
{
    ros::NodeHandle nh("~");

    bool success = true;

	// Install the symlink file in the symlink-mm folder to use the /dev/MicroMaestro alias
    nh.param<string>("device_name", deviceName, "/dev/botboard");
    nh.param<int>("baud_rate", baudRate, 115200);
    nh.param<int>("rate_hz", rate_hz, 50); //replaces 20 msec looptime

    //Set up the sensor descriptor.
    sd.setId(DESCRIPTOR_BOTBOARD);
    sd.setType(PSYNC_SENSOR_KIND_NOT_AVAILABLE);

    //Attempt to create serial device.
	try {
		serialInterface = RPM::SerialInterface::createSerialInterface( deviceName, baudRate , &errorMessage );
		if ( !serialInterface )
		{	
			// Log Error from the RPM library
			ROS_ERROR("Failed to create serial interface:");
			cout <<"Failed to create serial interface\n\r";
			//ROS_ERROR(errorMessage.c_str()); //where is errorMesssage declared? BotBoard header
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
/*
//Register listeners for message types here.
void BotBoard::initStateEvent()
{

    //Attempt to create serial device.
    try
    {
        serialDevice = shared_ptr<Serial>(new Serial(deviceName));
        serialDevice->open();
        serialDevice->setDataRate(DATARATE_115200);
        //serialDevice->setNonblockOption(1);
        serialDevice->applySettings();

        deviceIsOpen = true;
        logDebug("Successfully opened serial device.");
        this_thread::sleep_for(chrono::seconds(1));
    }
    catch (exception& e)
    {
        ostringstream oss;
        oss << "Strange error occurred opening serial port. The error was: ";
        oss << e.what();
    }
}*/

// okStateEvent replaced by loop in main.cpp
/*
void BotBoard::okStateEvent()
{
    try
    {
        read_serial();
    }
    catch (exception& e)
    {
        readErrorCount += 1;
    }

    if (readErrorCount > acceptableReadErrors)
    {
        logError("The specified number of read errors has been exceeded.");
    }

    //Poll the interface every <loopTime> ms.
    this_thread::sleep_for(loopTime);
}*/

//Clean up resources in this function.
/*
void BotBoard::releaseStateEvent()
{
    if (deviceIsOpen)
    {
        logDebug("Closing serial device.");
        serialDevice->close();
    }
}*/

void BotBoard::read_serial()
{
    vector<unsigned char> shortReadBuffer;
    ostringstream debugStream;

    // Reading the raw serial data
    // Need replacement for std::shared_ptr<polysync::Serial> serialDevice;
    // Need replacement for ps_timestamp lastReadTime;
    // Try the cereal_port package, NOPE, just on fuerte
    // Try ros-kinetic-serial, already installed by default
    // https://github.com/wjwwood/serial/blob/master/examples/serial_example.cc
    //******************************************************************************
    // REPLACE
    unsigned int bytesRead = serialDevice->read(shortReadBuffer, lastReadTime);
    serialDevice->flush();
    // REPLACE END
    // reading the serial data from hardware is done at this point
    // ********************************************************************************************

    if (bytesRead > 0)
        readBuffer.insert(readBuffer.end(), shortReadBuffer.begin(), shortReadBuffer.end());
    
    while (find(readBuffer.begin(), readBuffer.end(), '\n') != readBuffer.end())
    {
        auto foundCrLf = find(readBuffer.begin(), readBuffer.end(), '\n');
        vector<unsigned char> charsToProcess;
        foundCrLf++;
        charsToProcess.insert(charsToProcess.end(), readBuffer.begin(), foundCrLf);
        readBuffer.erase(readBuffer.begin(), foundCrLf);

        bool readSuccess = false;
        ostringstream debugOss;
        unsigned int bytesRead = charsToProcess.size();

        debugOss << "Read bytes: " << bytesRead << "; ";

        for (unsigned int i = 0; i < bytesRead - 2; i++)
            debugOss << charsToProcess[i];

        debugOss << endl;

        logDebug(debugOss.str());

        // SHOULD BE AN EASY ROS REPLACEMENT
        // NEW msg NEEDED, should not need to be custom
        //Create a ByteArrayMessage for the low-level output.
        dm::ByteArrayMessage bam(*this);
        bam.setDataType(BOTBOARD_DATA);
        bam.setHeaderTimestamp(getTimestamp());
        bam.setBytes(charsToProcess);
        bam.publish();
        // ROS REPLACE END
        
        //Parse the read values into the appropriate messages.
        vector<string> readStrings;
        ostringstream oss;

        //Ignore starting character ("U") and last two bytes (cr/lf).
        for (unsigned int i = 1; i < bytesRead - 2; ++i)
        {
            if (charsToProcess[i] != ',')
            {
                oss << charsToProcess[i];
            }
            else
            {
                readStrings.push_back(oss.str());
                oss.str("");
                oss.clear();
            }
        }

        readStrings.push_back(oss.str());
        oss.str("");
        oss.clear();
        
        // organizing the serial data is done at this point
        // ********************************************************************************************
        
        // Now we process the serial data, readStrings, and advertise the speedMs and steerCurve to ROS nodes
        // You already have created the speed and curvature message. Vehicle Model subscribes to it and will receive it from BotBoard
        if (readStrings.size() == 14)
        {
            dm::PlatformControlMessage pcm(*this);
            pcm.setSensorDescriptor(sd);
            pcm.setHeaderTimestamp(getTimestamp());
            pcm.setDestGuid(PSYNC_GUID_INVALID);
            
            unsigned long overridePulse = strtoul(readStrings[COMPUTER_OVERRIDE_PULSE].c_str(), NULL, 10);
            float speedMs;
            float steerCurve;

            if (overridePulse < 1300) {
                //RC Mode
                float normSpeed = normalizePWM(strtoul(readStrings[SPEED_PULSE].c_str(), NULL, 10));
                float normSteer = normalizePWM(strtoul(readStrings[STEER_PULSE].c_str(), NULL, 10));

                speedMs = normSpeed * 8.0;
                steerCurve = -normSteer * (1.0 / minCurve);

                pcm.setSpeed(speedMs);
                pcm.setCurvature(steerCurve);
                pcm.setTimestamp(getTimestamp());
                pcm.publish();
            } else if (overridePulse > 1700) {
                //Autonomous Mode
            } else {
                //Pause Mode
                speedMs = 0.0;
                steerCurve = 0.0;

                pcm.setSpeed(speedMs);
                pcm.setCurvature(steerCurve);
                pcm.setTimestamp(getTimestamp());
                pcm.publish();
            }
                
            readSuccess = true;
        }
        else
        {
            readErrorCount += 1;
            logError("Received incorrect number of parameters.");
        }

        //Reset the readErrorCount to 0 on successful read.
        if (readSuccess)
        {
            readErrorCount = 0;
        }
    }
}

float BotBoard::normalizePWM(unsigned long pwm)
{
    if (pwm == 0)
        return 0;

    pwm = min((unsigned long)PWM_MAX, pwm);
    pwm = max((unsigned long)PWM_MIN, pwm);
    
    if (pwm > PWM_CENTER && pwm < PWM_CENTER + PWM_DEADBAND) pwm = PWM_CENTER;
    if (pwm < PWM_CENTER && pwm > PWM_CENTER - PWM_DEADBAND) pwm = PWM_CENTER;

    if (pwm > PWM_CENTER)
    {
        return (((float)pwm - PWM_CENTER) / (PWM_MAX - PWM_CENTER));
    }
    else
    {
        return (((float)pwm - PWM_CENTER) / (PWM_CENTER - PWM_MIN));
    }
}
