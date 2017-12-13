#ifndef SERVO_HPP
#define SERVO_HPP

    //# Maestro channel outputs
    //# right-front steering servo = 0
    //# right-rear steering servo = 1
    //# left-front steering servo = 2
    //# left-rear steering servo = 3
    //# left-side motor ESC = 4
    //# right-side motor ESC = 5


namespace CIRC
{
	class Servo
	{
		public:
			enum MMChannel{
				STEER_RF = 0,
				STEER_RR = 1,
				STEER_LF = 2,
				STEER_LR = 3,
				ESC_R = 4,
				ESC_L = 5, } ;
			void setPWM(unsigned short Min, unsigned short Center, unsigned short Max);
			void setChannel(MMChannel ChannelIn);
			void setCommand(unsigned short Cmd);
			unsigned short getCenter();
			unsigned short getMin();
			unsigned short getMax();
			unsigned short getCommand();
			unsigned char getChannel();
		private:
				unsigned short Center_PWM, Min_PWM, Max_PWM, Command;
				MMChannel Channel;

	};


}
#endif
