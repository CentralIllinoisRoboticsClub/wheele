#include "ros_micromaestro/MicroMaestro.hpp"
//#include <iostream>
//#include <unistd.h>
//#include <signal.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MicroMaestro");
	//ros::init(argc, argv, "MicroMaestro", ros::init_options::NoSigintHandler);
    
    CIRC::MicroMaestro MicMas; //MicMas() with parentheses causes member functions to not be found

	// CONSIDER MAKING initialize() private and calling it in the constructor, not here
	// Then just call ros::spin() below and comment out the rate and while loop
	// Perhaps in order to safely close the serial connection the while loop is needed
	if(!MicMas.initialize())
    {
        ROS_INFO("Failed to initialize MicroMaestro node");
        return EXIT_FAILURE;
    }
    else
    {
        ROS_INFO("Successfully initialized MicroMaestro node");
    }

    //ros::spin();

    ros::Rate rate(MicMas.get_rate_hz());
	//signal(SIGINT, MicMas.~MicroMaestro()); //add a public function that then calls the destructor
		//Probably needs to return an int, not void

	while(ros::ok())
	{
		ros::spinOnce();
        rate.sleep();
		/*try
		{
			ros::spinOnce();
        	rate.sleep();
		}
		catch(...)
		{MicMas.~MicroMaestro();}
		*/
	}

	MicMas.~MicroMaestro(); //Why did ros_pololu_controller comment this out?

	ROS_INFO("Exited MicroMaestro node");

	return EXIT_SUCCESS;
}
