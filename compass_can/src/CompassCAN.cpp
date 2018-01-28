#include "CompassCAN.h"

/**********************************************************************
* Leddar CAN communication to ROS
* Requires:
* sudo apt install ros-kinetic-ros-opencan
* rosrun socketcan_bridge socketcan_bridge_node
*   The socket_can bridge turns CAN into /received_messages topic
*   The socket_can bridge turns /sent_messages into CAN 
* 
* Publish 740#01 at a specified frequency or just 740#02 once to request Leddar scan data
* Subscribe to /received_messages and respond to Leddar CAN ids 751 and 750
* 
* The leddar.launch file launches this node as well as a static transform between base_link and laser
**********************************************************************/

//Constructor
CompassCAN::CompassCAN()
{
    //Topics you want to publish
    heading_pub_ = nh_.advertise<std_msgs::Float32>("bno_heading", 10); //send heading

    //Topic you want to subscribe
    // leddarCallback will run every time ros sees the topic /received_messages
    can_sub_ = nh_.subscribe("received_messages", 50, &CompassCAN::compassCallback, this); //receive CAN
}

//Default destructor.
CompassCAN::~CompassCAN()
{
}

void CompassCAN::compassCallback(const can_msgs::Frame& frame)
{    
    //can id
    uint32_t id = frame.id;
    
    // look for 0x751 to start a scan reading
    if(id == 0x133)
    {
        float raw_heading = (float)(frame.data[0]<<8 | frame.data[1]);
        heading.data = (raw_heading-32768)/100.0;
        heading_pub_.publish(heading);
    }
}

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "compass_can");

    CompassCAN compass_can;
    ROS_INFO("Starting Compass CAN");
    int loop_rate = 10;
    ros::Rate rate(loop_rate);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
