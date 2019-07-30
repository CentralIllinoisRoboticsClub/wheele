#include "NavStates.h"
#include "AvoidObsCommon.h" // get_yaw()
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <boost/math/special_functions/round.hpp>
#include <algorithm>

/**********************************************************************
 * State machine for navigating to waypoints and perception targets
 *
 * Subscribe to /scan, /odom, /path and Publish /cmd_vel, /wp_goal
 **********************************************************************/

#define STATE_RETREAT_FROM_CONE -2
#define STATE_RETREAT -1
#define STATE_TRACK_PATH 0
#define STATE_TURN_TO_TARGET 1
#define STATE_TOUCH_TARGET 2
#define STATE_SEARCH_IN_PLACE 3

//Constructor
NavStates::NavStates() :
m_collision(false),
m_cone_detected(false),
m_odom_received(false),
m_path_received(false),
m_init_wp_published(false),
m_bump_switch(0),
m_num_waypoints(0),
m_index_wp(0),
m_state(STATE_TRACK_PATH),
m_index_path(0),
m_speed(0.0),
m_omega(0.0),
m_scan_collision_db_count(0),
m_cone_detect_db_count(0)
{
  //Topics you want to publish
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",10);

  wp_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("wp_goal",1);
  wp_cone_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("wp_cone_pose",1); //temp, Need to send this to avoid_obs to clear the costmap at cone
  nav_state_pub_ = nh_.advertise<std_msgs::Int16>("nav_state",1);

  //Topic you want to subscribe
  scan_sub_ = nh_.subscribe("scan", 50, &NavStates::scanCallback, this); //receive laser scan
  odom_sub_ = nh_.subscribe("odom", 10, &NavStates::odomCallback, this);
  clicked_goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &NavStates::clickedGoalCallback, this);
  cam_cone_pose_sub_ = nh_.subscribe("cam_cone_pose", 1, &NavStates::camConeCallback, this);
  bump_sub_ = nh_.subscribe("bump_switch",1, &NavStates::bumpCallback, this);
  path_sub_ = nh_.subscribe("path",5, &NavStates::pathCallback, this);
  nh_p  = ros::NodeHandle("~");
  nh_p.param("plan_rate_hz", params.plan_rate, 10.0);
  nh_p.param("use_PotFields", params.use_PotFields, false);
  nh_p.param("close_cone_to_bot_dist", params.close_cone_to_bot_dist, 6.0);
  nh_p.param("valid_cone_to_wp_dist", params.valid_cone_to_wp_dist, 1.0);
  nh_p.param("near_path_dist", params.near_path_dist, 1.0);
  nh_p.param("desired_speed", params.desired_speed, 0.6);
  nh_p.param("slow_speed", params.slow_speed, 0.3);
  nh_p.param("max_omega", params.max_omega, 0.5);
  nh_p.param("max_fwd_heading_error_deg", params.max_fwd_heading_error_deg, 90.0);
  nh_p.param("search_time", params.search_time, 1.0);
  nh_p.param("search_omega", params.search_omega, 0.1);
  nh_p.param("reverse_time", params.reverse_time, 2.0);
  nh_p.param("cmd_control_ver", params.cmd_control_ver, 0);
  nh_p.param("scan_collision_db_limit", params.scan_collision_db_limit, 2);
  nh_p.param("cone_detect_db_limit", params.cone_detect_db_limit, 2);
  nh_p.param("cmd_speed_filter_factor", params.cmd_speed_filter_factor, 0.5);

  nh_p.param("x_coords", x_coords, x_coords);
  nh_p.param("y_coords", y_coords, y_coords);
  nh_p.param("waypoints_are_in_map_frame", waypoints_are_in_map_frame, true);

  //listener.setExtrapolationLimit(ros::Duration(0.1));
  listener.waitForTransform("laser", "odom", ros::Time(0), ros::Duration(10.0)); //TODO: ros::Time(0) or ::now() ??
  listener.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(10.0));

  if(waypoints_are_in_map_frame)
  {
    map_goal_pose.header.frame_id = "map";
  }
  else
  {
    map_goal_pose.header.frame_id = "odom";
  }
  bot_pose.header.frame_id = "odom";
  map_goal_pose.pose.orientation.w = 1.0;
  bot_pose.pose.orientation.w = 1.0;
  odom_goal_pose = bot_pose;
  bot_yaw = 0.0;

  camera_cone_pose.header.frame_id = "odom";
  obs_cone_pose.header.frame_id = "odom";
  camera_cone_pose.pose.orientation.w = 1.0;
  obs_cone_pose.pose.orientation.w = 1.0;

  m_num_waypoints = x_coords.size();
  for(unsigned k=0; k<m_num_waypoints; ++k)
  {
    geometry_msgs::Point wp; wp.x = x_coords[k]; wp.y = y_coords[k];
    m_waypoints.push_back(wp);
  }
  m_index_wp = 0;
  update_waypoint();

  m_state = STATE_TRACK_PATH;

  state_init();

  ROS_INFO("Initialized Navigation State Manager");
}

NavStates::~NavStates(){}

void NavStates::update_waypoint()
{
  map_goal_pose.header.stamp = ros::Time::now();
  map_goal_pose.pose.position.x = m_waypoints[m_index_wp].x;
  map_goal_pose.pose.position.y = m_waypoints[m_index_wp].y;
  ROS_INFO("Update Waypoint");
  ROS_INFO("map x,y = %0.1f, %0.1f",map_goal_pose.pose.position.x, map_goal_pose.pose.position.y);
  geometry_msgs::PoseStamped odom_pose;
  odom_pose.header.frame_id = "odom";
  if(getPoseInFrame(map_goal_pose, "odom", odom_pose))
  {
    odom_goal_pose = odom_pose;
    ROS_INFO("odom x,y = %0.1f, %0.1f",odom_goal_pose.pose.position.x, odom_goal_pose.pose.position.y);
    ++m_index_wp;
    if(m_index_wp == m_num_waypoints)
    {
      m_index_wp = 0;
    }
    wp_goal_pub_.publish(odom_goal_pose);
    camera_cone_pose = odom_goal_pose;
    wp_cone_pub_.publish(camera_cone_pose); // publish this once to avoid obs so it will clear the costmap at the initial goal, until it finds an updated goal from the camera
    ROS_INFO("wp_goal published");
    m_init_wp_published = true;
    m_cone_detect_db_count = 0;
  }
}

void NavStates::pathCallback(const nav_msgs::Path& path_in)
{
  m_path = path_in;
  m_path_received = true;
  m_index_path = 0;
}

void NavStates::camConeCallback(const geometry_msgs::PoseStamped& cone_pose_in)
{
  if(m_state != STATE_RETREAT_FROM_CONE)
  {
    geometry_msgs::PoseStamped cone_in_map;
    if(waypoints_are_in_map_frame)
    {
      cone_in_map.header.frame_id = "map";
    }
    else
    {
      cone_in_map.header.frame_id = "odom";
    }
    if(getPoseInFrame(cone_pose_in, cone_in_map.header.frame_id, cone_in_map))
    {
      if(distance_between_poses(cone_in_map, map_goal_pose) < params.valid_cone_to_wp_dist)
      {
        geometry_msgs::PoseStamped cone_in_odom;
        cone_in_odom.header.frame_id = "odom";
        if(getPoseInFrame(cone_pose_in, "odom", cone_in_odom) )
        {
          ++m_cone_detect_db_count;
          if( (distance_between_poses(cone_in_odom, bot_pose) < params.close_cone_to_bot_dist)
              && (m_cone_detect_db_count >= params.cone_detect_db_limit) )
          {
            camera_cone_pose = cone_in_odom;
            wp_cone_pub_.publish(camera_cone_pose);
            m_cone_detect_db_count = params.cone_detect_db_limit;
            m_cone_detected = true;
          }
        }
        else if(m_cone_detect_db_count > 0)
        {
          --m_cone_detect_db_count;
        }
      }
    }
  }
}

double NavStates::distance_between_poses(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
{
  double dx = pose1.pose.position.x - pose2.pose.position.x;
  double dy = pose1.pose.position.y - pose2.pose.position.y;
  return sqrt(dx*dx + dy*dy);
}

bool NavStates::getPoseInFrame(const geometry_msgs::PoseStamped& pose_in, std::string target_frame,
    geometry_msgs::PoseStamped& pose_out)
{
  // *********** tf LESSONS LEARNED ****************
  // waitForTrasform req_time = timestamp of the pose we are transforming
  //   do NOT use ros::Time(0) or ros::Time::now()
  //   TODO: Compare to AvoidObs::scanCallback()

  //pose_in.header.stamp = ros::Time::now(); //TODO: does this help or hurt?
  ros::Time req_time = pose_in.header.stamp; // - ros::Duration(0.1);
  //pose_out.header.stamp = req_time;
  //TODO: ros::Time(0) or ::now() or pose_in stamp ?? CHECK IF pose_in stamp is set to recent outside of this function
  listener.waitForTransform(target_frame, pose_in.header.frame_id, req_time, ros::Duration(10.0));
  try
  {
    listener.transformPose(target_frame, pose_in, pose_out);
    return true;
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("NavStates getPose Received an exception trying to transform a pose : %s", ex.what());
    return false;

  }
}

void NavStates::bumpCallback(const std_msgs::Int16& msg)
{
  m_bump_switch = msg.data;
}

double NavStates::get_plan_rate()
{
  return params.plan_rate;
}

void NavStates::odomCallback(const nav_msgs::Odometry& odom)
{
  bot_pose.pose.position = odom.pose.pose.position;
  bot_pose.pose.orientation = odom.pose.pose.orientation;
  bot_yaw = get_yaw(bot_pose.pose);
  m_odom_received = true;
}

void NavStates::clickedGoalCallback(const geometry_msgs::PoseStamped& data)
{
  geometry_msgs::PoseStamped odomPose;
  odomPose.header.frame_id = "odom";
  if(getPoseInFrame(data,"odom",odomPose))
  {
    odom_goal_pose = odomPose;
    wp_goal_pub_.publish(odom_goal_pose);
    ROS_INFO("Updated wp_goal via clicked point");
  }
}

void NavStates::scanCallback(const sensor_msgs::LaserScan& scan) //use a point cloud instead, use laser2pc.launch
{
  m_collision = false;
  unsigned close_count = 0;

  for (int i = 0; i < scan.ranges.size();i++)
  {
    float range = scan.ranges[i];
    float angle  = scan.angle_min +(float(i) * scan.angle_increment);
    if(range < 0.5)
    {
      ++close_count;
    }
  }
  if(close_count > 2)
  {
    ++m_scan_collision_db_count;
    if(m_scan_collision_db_count > params.scan_collision_db_limit)
    {
      m_scan_collision_db_count = params.scan_collision_db_limit;
      m_collision = true;
    }
  }
  else
  {
    m_scan_collision_db_count = 0;
  }
}

bool NavStates::nearPathPoint()
{
  return distance_between_poses(bot_pose, m_path.poses[m_index_path]) < params.near_path_dist;
}

//getTargetHeading assumes goal is in odom frame
double NavStates::getTargetHeading(const geometry_msgs::PoseStamped& goal)
{
  return atan2(goal.pose.position.y - bot_pose.pose.position.y, goal.pose.position.x - bot_pose.pose.position.x);
}

void NavStates::commandTo(const geometry_msgs::PoseStamped& goal)
{
  m_speed = params.desired_speed;
  double des_yaw = getTargetHeading(goal);

  if(params.cmd_control_ver == 0)
  {
    // ********** FROM DiffDriveController ********
    double ka= 2.0;
    double kb= 0.001;
    double theta = bot_yaw;
    double pos_beta = des_yaw;
    double alpha = pos_beta - theta;
    if(alpha >= M_PI)
      alpha -= 2*M_PI;
    else if(alpha < -M_PI)
      alpha += 2*M_PI;

    m_omega = ka*alpha + kb*pos_beta;
    // **********************************************
  }
  else
  {
    m_omega = 0.5*(des_yaw - bot_yaw);
    if(fabs(m_omega) > 1.0)
    {
      m_omega = 1.0*m_omega/fabs(m_omega);
      m_speed = 0.4;
    }
  }
  if(fabs(des_yaw - bot_yaw) > params.max_fwd_heading_error_deg*M_PI/180)
  {
    m_speed = -m_speed;
  }

  //ROS_INFO("des_yaw, bot_yaw, omega: %0.1f, %0.1f, %0.1f",des_yaw*180/M_PI, bot_yaw*180/M_PI, m_omega);
}

void NavStates::state_init()
{
  state_start_time = ros::Time::now();
}
double NavStates::get_time_in_state()
{
  return (ros::Time::now() - state_start_time).toSec();
}

void NavStates::track_path()
{
  if(!m_init_wp_published)
  {
    update_waypoint();
  }

  if(!(m_odom_received && m_path_received) || m_path.poses.size() == 0)
  {
    ROS_INFO("odom_received, path_received, path size: %d, %d, %d",(int)m_odom_received, (int)m_path_received, (int)m_path.poses.size());
    //m_state = STATE_SEARCH_IN_PLACE;
    return;
  }

  if(m_collision)
  {
    m_state = STATE_RETREAT;
    m_speed = 0.0;
    m_omega = 0.0;
    return;
  }

  if(m_cone_detected)
  {
    m_state = STATE_TURN_TO_TARGET;
    return;
  }

  // end of path?
  if(m_index_path == m_path.poses.size()-1)
  {
    m_state = STATE_SEARCH_IN_PLACE;
    return;
  }
  //check if close to next path point and update
  if(nearPathPoint())
  {
    m_index_path += 3; //TODO: PARAMETER
    if(m_index_path >= m_path.poses.size())
    {
      m_index_path = m_path.poses.size()-1;
    }
  }
  geometry_msgs::PoseStamped path_pose = m_path.poses[m_index_path];
  commandTo(path_pose);
}
void NavStates::retreat()
{
  m_speed = -params.desired_speed;
  m_omega = 0.0; //eventually retreat with m_omega = omega from path control
  if(get_time_in_state() > params.reverse_time)
  {
    m_speed = 0.0;
    m_state = STATE_TRACK_PATH;
  }
}
void NavStates::search_in_place()
{
  m_speed = params.slow_speed;
  m_omega = params.search_omega;
  if(m_cone_detected)
  {
    m_state = STATE_TOUCH_TARGET;
    update_target();
  }
  // If still at end of path, it will come right back here from STATE_TRACK_PATH
  if(get_time_in_state() > params.search_time || m_collision)
  {
    m_state = STATE_TRACK_PATH;
  }
}
void NavStates::turn_to_target()
{
  double des_yaw = getTargetHeading(camera_cone_pose);
  m_speed = params.slow_speed;
  m_omega = 1.0*(des_yaw - bot_yaw);

  if(fabs(m_omega) > params.max_omega) // TODO: PARAMETER
    m_omega = params.max_omega*m_omega/fabs(m_omega); //TODO: PARAMETER
  if(fabs(m_omega) < params.search_omega)
    m_omega = params.search_omega*m_omega/fabs(m_omega); //TODO: PARAMETER
  if(m_cone_detected)
  {
    m_state = STATE_TOUCH_TARGET;
    update_target();
    return;
  }
  if(fabs(des_yaw - bot_yaw) < 5*M_PI/180 || get_time_in_state() > 3.0) //TODO: PARAMETER
  {
    m_state = STATE_TRACK_PATH;
  }
}
void NavStates::touch_target()
{
  commandTo(camera_cone_pose);
  if(m_bump_switch > 0 || m_collision)
  {
    // this causes update_waypoint() to be called again until is is successful
    m_init_wp_published = false;
    m_cone_detected = false;
    m_cone_detect_db_count = 0;

    update_waypoint();
    m_speed = 0.0;
    m_omega = 0.0;
    m_state = STATE_RETREAT_FROM_CONE;
  }
}
void NavStates::retreat_from_cone()
{
  // NOTE, this state should block target updates
  //   Do not allow the just touched cone to revert the updated waypoint

  if(!m_init_wp_published)
  {
    update_waypoint();
  }

  m_speed = -params.desired_speed;
  m_omega = 0.0;
  if(get_time_in_state() > params.reverse_time)
  {
    m_speed = 0.0;
    m_state = STATE_TRACK_PATH;
  }
}
void NavStates::update_target()
{
  wp_goal_pub_.publish(camera_cone_pose);
}

void NavStates::update_states()
{
  int prev_state = m_state;
  switch(m_state) {
  case STATE_TRACK_PATH:
    track_path();
    break;
  case STATE_TURN_TO_TARGET:
    turn_to_target();
    break;
  case STATE_TOUCH_TARGET:
    touch_target();
    break;
  case STATE_SEARCH_IN_PLACE:
    search_in_place();
    break;
  case STATE_RETREAT:
    retreat();
    break;
  case STATE_RETREAT_FROM_CONE:
    retreat_from_cone();
    break;
  default: //optional
  ROS_INFO("Nav State not defined");
  }

  if(prev_state != m_state)
  {
    state_init();
  }

  if(!(m_odom_received && m_path_received) )
  {
    m_speed = 0.0;
    m_omega = 0.0;
  }

  if(fabs(m_omega) > params.max_omega)
  {
    m_omega = params.max_omega*m_omega/fabs(m_omega);
    m_speed = params.slow_speed*m_speed/fabs(m_speed);
  }

  if(m_speed <= 0.0)
  {
    m_filt_speed = m_speed;
  }
  else
  {
    double alpha = params.cmd_speed_filter_factor;
    m_filt_speed = m_filt_speed*alpha + m_speed*(1.0-alpha);
  }

  geometry_msgs::Twist cmd;
  cmd.linear.x = m_filt_speed;
  cmd.angular.z = m_omega;
  cmd_pub_.publish(cmd);

  m_nav_state_msg.data = m_state;
  nav_state_pub_.publish(m_nav_state_msg);
}

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "nav_states");

  NavStates nav_states;
  ROS_INFO("Starting Navigation State Manager");
  int loop_hz = nav_states.get_plan_rate();
  ros::Rate rate(loop_hz);

  while(ros::ok())
  {
    ros::spinOnce();
    nav_states.update_states();
    rate.sleep();
  }

  return 0;
}
