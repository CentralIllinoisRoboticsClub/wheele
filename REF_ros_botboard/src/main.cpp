#include "BotBoard.hpp"
//#include <iostream>
//#include <unistd.h>
//#include <signal.h>

int main(int argc, char* argv[])
{
  //Initiate ROS
  ros::init(argc, argv, "BotBoard");

  //Create an object of VehicleModel that will take care of everything
  CIRC::BotBoard bb;

	// CONSIDER MAKING initialize() private and calling it in the constructor, not here
	// Then just call ros::spin() below and comment out the rate and while loop
	// Perhaps in order to safely close the serial connection the while loop is needed
	if(!bb.initialize())
    {
        ROS_INFO("Failed to initialize BotBoard node");
        return EXIT_FAILURE;
    }
    else
    {
        ROS_INFO("Successfully initialized BotBoard node");
    }

    //ros::spin();

    ros::Rate rate(bb.get_rate_hz());
	//signal(SIGINT, bb.~BotBoard()); //add a public function that then calls the destructor
		//Probably needs to return an int, not void

	while(ros::ok())
	{
		ros::spinOnce();
        rate.sleep();
	}

	bb.~BotBoard(); //Why did ros_pololu_controller comment this out?

	ROS_INFO("Exited BotBoard node");

	return EXIT_SUCCESS;
}
