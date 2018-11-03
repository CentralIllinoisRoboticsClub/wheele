#include "AvoidObs.h"
#include <geometry_msgs/PointStamped.h>
#include <math.h>

/**********************************************************************
* Obstacle Avoidance using a nav_msgs/OccupacyGrid and A* path planning
* 
* Subscribe to /scan and Publish OccupacyGrid /costmap, Publish Path /path
* 
* The leddar.launch file launches this node as well as a static transform between base_link and laser
**********************************************************************/

//Constructor
AvoidObs::AvoidObs()
{
    //Topics you want to publish
    costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("costmap", 1);
    path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1);

    //Topic you want to subscribe
    // leddarCallback will run every time ros sees the topic /received_messages
    scan_sub_ = nh_.subscribe("scan", 50, &AvoidObs::scanCallback, this); //receive laser scan
    odom_sub_ = nh_.subscribe("odom", 10, &AvoidObs::odomCallback, this);
    nh_p  = ros::NodeHandle("~");
    nh_p.param("plan_rate_hz", plan_rate_, 1); //set in avoid_obs.launch
    nh_p.param("map_res_m", map_res_, 0.5);
    nh_p.param("map_size", n_width_, 200);
    nh_p.param("map_size", n_height_, 200);
    nh_p.param("max_range", max_range_, 40.0);
    nh_p.param("plan_range_m",plan_range_, 40.0);
    ROS_INFO("map_size (n cells): %d", n_width_);
    
    //listener.setExtrapolationLimit(ros::Duration(0.1));
    listener.waitForTransform("laser", "odom", ros::Time(0), ros::Duration(10.0));

    num_obs_cells = 0; //number of obstacle cells
    
    map_pose.position.x = -n_width_*map_res_/2; // I believe this is zero by default, check by echoing costmap
    map_pose.position.y = -n_height_*map_res_/2; // will need to update x and y as we move
    map_pose.orientation.w = 1.0;
    
    costmap.header.stamp = ros::Time::now();
    costmap.header.frame_id = "odom";
    costmap.info.resolution = map_res_;
    costmap.info.width = n_width_;
    costmap.info.height = n_height_;
    costmap.info.origin = map_pose;
    // Fill costmap with zeros
    // cost(ix,iy) = costmap.data[ix*n_height + iy], x-RIGHT, y-UP, 0,0 is bottom left
    costmap.data.resize(n_width_*n_height_);
    
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "odom";
    //path.poses.push_back(geometry_msgs::PoseStamped)

    //test Astar setup
    geometry_msgs::Pose start, goal;
    start.position.x = 0;
    start.position.y = 0;
    start.orientation.w = 1.0;
    goal.position.x = 20;
    goal.position.y = -38;
    goal.orientation.w = 1.0;
    astar.get_path(start, goal, costmap, path);

}

AvoidObs::~AvoidObs(){}

int AvoidObs::get_plan_rate()
{
    return plan_rate_;
}

void AvoidObs::update_cell(float x, float y, int val)
{
	float x0 = map_pose.position.x;
	float y0 = map_pose.position.y;
	int ix = (x-x0)/map_res_;
	int iy = (y-y0)/map_res_;
	if(0 <= ix && ix < n_width_ && 0 <= iy && iy < n_height_)
	{
		costmap.data[iy*n_width_ + ix] += val;
		if(costmap.data[iy*n_width_ + ix] > 100)
			costmap.data[iy*n_width_ + ix] = 100;
		else if(costmap.data[iy*n_width_ + ix] < 0)
			costmap.data[iy*n_width_ + ix] = 0;
	}
}

bool AvoidObs::update_plan()
{
	/*path.poses.clear();
	geometry_msgs::PoseStamped wp;
	wp.pose.position.x = 5.0;
	wp.pose.position.y = 5.0;
	path.poses.push_back(wp);
	*/

	geometry_msgs::Pose start, goal;
	start.position.x = 0;
	start.position.y = 0;
	start.orientation.w = 1.0;

	goal.position.x = -40;
	goal.position.y = 40;
	goal.orientation = bot_pose.orientation; //we currently ignore goal orientation

	float dx = goal.position.x - bot_pose.position.x;
	float dy = goal.position.y - bot_pose.position.y;
	float goal_dist_sqd = dx*dx + dy*dy;
	if(goal_dist_sqd > plan_range_*plan_range_)
	{
		float dir_rad = atan2(dy,dx);
		goal.position.x = bot_pose.position.x+plan_range_*cos(dir_rad);
		goal.position.y = bot_pose.position.y+plan_range_*sin(dir_rad);
	}

	astar.get_path(bot_pose, goal, costmap, path);
	path.header.stamp = ros::Time::now();
	path_pub_.publish(path);
}

void AvoidObs::odomCallback(const nav_msgs::Odometry& odom)
{
	bot_pose.position = odom.pose.pose.position;
	bot_pose.orientation = odom.pose.pose.orientation;
}

void AvoidObs::scanCallback(const sensor_msgs::LaserScan& scan) //use a point cloud instead, use laser2pc.launch
{
	// Transform scan to map frame, clear and fill costmap

	geometry_msgs::PointStamped laser_point, odom_point;
	laser_point.header.frame_id = "laser";
	laser_point.header.stamp = scan.header.stamp;//ros::Time();
	laser_point.point.z = 0;
	
	for (int i = 0; i < scan.ranges.size();i++)
	{
	    float range = scan.ranges[i];
	    float angle  = scan.angle_min +(i * scan.angle_increment);

	    //clear map cells
	    for(double r = 0.5; r < (range - map_res_/2); r += map_res_)
	    {
	    	double angle_step = r*scan.angle_increment/map_res_;
	    	for(double a=(angle-scan.angle_increment/2); a < (angle+scan.angle_increment/2); a += angle_step)
	    	{
	    		laser_point.point.x = r*cos(a);
	    		laser_point.point.y = r*sin(a);
	    		try{
					listener.transformPoint("odom", laser_point, odom_point);
					update_cell(odom_point.point.x, odom_point.point.y, -5);
				}
				catch(tf::TransformException& ex){
					int xa;
					//ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
				}

	    	}
	    }

	    // fill obstacle cells
	    if(range < max_range_)
	    {
			laser_point.point.x = range*cos(angle) ;
			laser_point.point.y = range*sin(angle) ;

			try{
				listener.transformPoint("odom", laser_point, odom_point);
				update_cell(odom_point.point.x, odom_point.point.y, 5);
			}
			catch(tf::TransformException& ex){
				int xa;
				//ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
			}
	    }
	}

	//odom_point.point.x = 10.0;
	//odom_point.point.y = -5.0;
	//update_cell(odom_point.point.x, odom_point.point.y, 100.0);
	costmap_pub_.publish(costmap);
}

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "avoid_obs");

    AvoidObs avoid_obs;
    ROS_INFO("Starting Obstacle Avoidance");
    int loop_hz = avoid_obs.get_plan_rate();
    ros::Rate rate(loop_hz);

    while(ros::ok())
    {
        ros::spinOnce();
        avoid_obs.update_plan();
        rate.sleep();
    }

    return 0;
}
