// Copyright 2019 coderkarl. Subject to the BSD license.

#include "ImuCAN.h"
//#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

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

#define GYRO_CAN_ID 0x131
#define ACCEL_CAN_ID 0x132
#define COMPASS_CAN_ID 0x133

//Constructor
ImuCAN::ImuCAN()
{
    //Topics you want to publish
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 10);
    mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("magXYZ", 10);
    heading_pub_ = nh_.advertise<sensor_msgs::Imu>("mag_imu",10);

    //Topic you want to subscribe
    // leddarCallback will run every time ros sees the topic /received_messages
    can_sub_ = nh_.subscribe("received_messages", 50, &ImuCAN::imuCallback, this); //receive CAN
    
    accel.x = 0; accel.y = 0; accel.z = 9.81; //NEED TO VERIFY SIGN, FOLLOW ENU CONVENTION
    
    double accel_var = (4.0 * 1e-3f * 9.80665)*(4.0 * 1e-3f * 9.80665); //(m/s/s)^2
    imu.linear_acceleration_covariance[0] = accel_var;
    imu.linear_acceleration_covariance[4] = accel_var;
    imu.linear_acceleration_covariance[8] = accel_var;
    
    double gyro_var = (0.06 * 3.14159 / 180.0)*(0.06 * 3.14159 / 180.0); //(rad)^2
    imu.angular_velocity_covariance[0] = gyro_var;
    imu.angular_velocity_covariance[4] = gyro_var;
    imu.angular_velocity_covariance[8] = gyro_var;
    
    double mag_var = (3.0)*(3.0); //microTesla^2, even though it says use Tesla, they only calculate ratio anyway
    magXYZ.magnetic_field_covariance[0] = mag_var;
    magXYZ.magnetic_field_covariance[4] = mag_var;
    magXYZ.magnetic_field_covariance[8] = mag_var;
    
    imu.orientation.x = 0.0;
    imu.orientation.y = 0.0;
    imu.orientation.z = 0.0;
    imu.orientation.w = 1.0;
    imu.orientation_covariance[0] = -1;
    imu.orientation_covariance[4] = -1;
    imu.orientation_covariance[8] = -1;
    
    compass_heading = 0;
    mag_imu.orientation_covariance[8] = (13*3.14159/180.)*(13*3.14159/180.); //yaw about z
    
    imu.header.frame_id = "imu";
    mag_imu.header.frame_id = "imu";
    magXYZ.header.frame_id = "imu";
    
    //NOTE, the covariances in these messages may be ignored, overwritten in the tools you will use:
    // https://answers.ros.org/question/248619/about-using-imu_tools-filters/
    // See robot_localization details
    //    https://answers.ros.org/question/273372/robot_localization-exploding-covariances-why/
          // This suggests I do need covariance defined in my input /imu message to robot_localization
}

//Default destructor.
ImuCAN::~ImuCAN()
{
}

int ImuCAN::convertCAN(const can_msgs::Frame& frame, float data_out[4], unsigned int nVars) //, unsigned int numBytesPerVar)
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

void ImuCAN::imuCallback(const can_msgs::Frame& frame)
{    
    //can id
    uint32_t id = frame.id;
    
    //canConvert common library function pending...
    float data_out[4];
    unsigned int nVars = 4;
    int res = convertCAN(frame, data_out, nVars);
    
    // ###### rotating imu 180 deg in this code with x = -x, y = -y... consider applying it in tf between imu and base_link
    //   Also heading - 3.14159
    
    if(id == COMPASS_CAN_ID)
    {
        //NEED TO HAVE IMU_CALIBRATION_STATUS CAN msg sent from arduino, if no compass cal, do not publish mag_pose
        //  probably remove heading from this CAN msg, group mag-or-compass cal status with heading into one CAN msg
        compass_heading = (data_out[3])/100.0; //centi-deg to deg
        mag_imu.header.stamp = frame.header.stamp;
        geometry_msgs::Quaternion q;
        q = tf::createQuaternionMsgFromYaw(compass_heading*3.14159/180. - 3.14159);
        //q.normalize();
        mag_imu.orientation = q;
        //tf::quaternionTFToMsg(q,mag_pose.pose.pose.orientation);
        heading_pub_.publish(mag_imu);
        
        //update x,y,z order in arduino to match our x,y,z? Or just do it here?
        magXYZ.header.stamp = frame.header.stamp;
        magXYZ.magnetic_field.x = -data_out[0]/100.0; //microTesla
        magXYZ.magnetic_field.y = -data_out[1]/100.0;
        magXYZ.magnetic_field.z = data_out[2]/100.0;
        
        mag_pub_.publish(magXYZ);
    }
    else if(id == ACCEL_CAN_ID)
    {
        imu.header.stamp = frame.header.stamp;
        //magXYZ.header.frame_id = 'none';
        accel.x = -data_out[0]/100.0; // m/s/s,
        accel.y = -data_out[1]/100.0;
        accel.z = data_out[2]/100.0;
    }
    else if(id == GYRO_CAN_ID) // publish imu here, sample and hold accel values
    {
        imu.header.stamp = frame.header.stamp;
        imu.angular_velocity.x = -data_out[0]/100.0*3.14159/180.; //rad/sec
        imu.angular_velocity.y = -data_out[1]/100.0*3.14159/180.;
        imu.angular_velocity.z = data_out[2]/100.0*3.14159/180.;        
        imu.linear_acceleration = accel;
        imu_pub_.publish(imu);
    }
}

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "imu_can");

    ImuCAN compass_can;
    ROS_INFO("Starting IMU CAN");
    int loop_rate = 10;
    ros::Rate rate(loop_rate);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
