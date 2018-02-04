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
    mag_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("bno_magXYZ", 10);

    //Topic you want to subscribe
    // leddarCallback will run every time ros sees the topic /received_messages
    can_sub_ = nh_.subscribe("received_messages", 50, &CompassCAN::compassCallback, this); //receive CAN
}

//Default destructor.
CompassCAN::~CompassCAN()
{
}

int CompassCAN::convertCAN(const can_msgs::Frame& frame, float data_out[4], unsigned int nVars) //, unsigned int numBytesPerVar)
{
    if(nVars > 4)
        nVars = 4;
    
    // change float[4] data_out to a std::vector<float>
    for(unsigned int k=0; k<nVars; ++k)
    {
        data_out[k] = (float)(frame.data[2*k]<<8 | frame.data[2*k+1]) - 32768; //0-1; 2-3; 4-5; 6-7
    }
    return 0;
}

void CompassCAN::compassCallback(const can_msgs::Frame& frame)
{    
    //can id
    uint32_t id = frame.id;
    
    // look for 0x751 to start a scan reading
    if(id == 0x133)
    {
        //canConvert common library function pending...
        float data_out[4];
        unsigned int nVars = 4;
        int res = convertCAN(frame, data_out, nVars);
        
        heading.data = (data_out[0])/100.0;
        heading_pub_.publish(heading);
        
        //update x,y,z order in arduino to match our x,y,z? Or just do it here?
        magXYZ.header.stamp = frame.header.stamp;
        magXYZ.vector.x = data_out[1]/100.0;
        magXYZ.vector.y = data_out[2]/100.0;
        magXYZ.vector.z = data_out[3]/100.0;
        mag_pub_.publish(magXYZ);
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