#include "ros_micromaestro/Servo.hpp"
using namespace CIRC;

// Command to populate the servo class with 
void Servo::setPWM(unsigned short Min, unsigned short Center, unsigned short Max){
	Center_PWM = Center;
	Min_PWM = Min;
	Max_PWM = Max;
}

void Servo::setChannel(MMChannel ChannelIn)
{
	Channel = ChannelIn;
}

void Servo::setCommand(unsigned short Cmd)
{
	Command = Cmd;
}

unsigned short Servo::getCenter()
{
	return Center_PWM;
}

unsigned short Servo::getMin()
{
	return Min_PWM;
}

unsigned short Servo::getMax()
{
	return Max_PWM;
}

unsigned char Servo::getChannel()
{
	// Convert the Channel to an unsigned char
	return (unsigned char)Channel;
}

unsigned short Servo::getCommand()
{
	return Command;
}
