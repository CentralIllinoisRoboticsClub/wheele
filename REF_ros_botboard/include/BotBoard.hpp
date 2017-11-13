#ifndef BOTBOARD_HPP
#define BOTBOARD_HPP

//C++ Includes
#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>
#include <algorithm>

//OS Includes
#include <unistd.h>

//PolySync Includes
#include <PolySyncNode.hpp>
#include <PolySyncMessage.hpp>
#include <PolySyncSerial.hpp>
#include <PolySyncDTCException.hpp>
#include <PolySyncDataModel.hpp>

namespace CIRC
{
    enum BBValue
    {
        COMPUTER_OVERRIDE = 0,
        BUMP1 = 1,
        BUMP2 = 2,
        ENC1 = 3,
        ENC2 = 4,
        COMPUTER_OVERRIDE_PULSE = 5,
        SPEED_PULSE = 6,
        STEER_PULSE = 7,
        BATT_VOLT_DIGITAL_VAL = 8,
        ULTRASONIC1 = 9,
        ULTRASONIC2 = 10,
        ULTRASONIC3 = 11,
        ULTRASONIC4 = 12,
        ULTRASONIC5 = 13
    };

    class BotBoard : public polysync::Node
    {
        public:
            BotBoard();
            ~BotBoard();
            bool initialize();
			double get_rate_hz();
        private:
            void read_serial();
            float normalizePWM(unsigned long pwm);
            bool outputTicks;
            std::string deviceName;
            //std::shared_ptr<polysync::Serial> serialDevice;
            //ps_timestamp lastReceivedTime;
            bool deviceIsOpen;
            const std::chrono::milliseconds loopTime = std::chrono::milliseconds(20);
            long readErrorCount;
            const unsigned short int acceptableReadErrors = 10;
            //polysync::datamodel::SensorDescriptor sd;
            //ps_timestamp lastReadTime;
            const unsigned long BOTBOARD_DATA = 99;
            //const ps_identifier BOTBOARD_SENSOR_ID = 2001;
            const float PWM_MIN = 1100.0;
            const float PWM_MAX = 1920.0;
            const float PWM_CENTER = 1500.0;
            const float PWM_DEADBAND = 10.0;
            const float minCurve = 0.6; //Minimum radius of curvature in m.
            std::vector<unsigned char> readBuffer;
            
            //From MicroMaestro example
			std::string errorMessage;
			RPM::SerialInterface* serialInterface;
    };
}

#endif
