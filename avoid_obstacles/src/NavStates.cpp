// Copyright 2019 coderkarl. Subject to the BSD license.

#include "avoid_obstacles/NavStates.h"
#include "avoid_obstacles/AvoidObsCommon.h" // get_yaw()
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

#define WP_TYPE_INTER 0
#define WP_TYPE_CONE 1

//Constructor
NavStates::NavStates() :
m_current_waypoint_type(WP_TYPE_INTER),
m_current_hill_type(0),
m_collision(false),
m_cone_detected(false),
m_odom_received(false),
m_path_received(false),
m_obs_cone_received(false),
m_init_wp_published(false),
m_odom_goal_refresh_needed(false),
m_first_search(true),
m_valid_bump(false),
m_bump_switch(0),
m_bump_count(0),
m_num_waypoints(0),
m_index_wp(0),
m_state(STATE_TRACK_PATH),
m_index_path(0),
m_speed(0.0),
m_omega(0.0),
m_filt_speed(0.0),
m_prev_speed(0.0),
m_scan_collision_db_count(0),
m_cone_detect_db_count(0),
m_close_to_obs(false)
{
  //Topics you want to publish
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",10);

  wp_static_map_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("static_map_goal",1);
  wp_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("wp_goal",1);
  wp_cone_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("wp_cone_pose",1); //temp, Need to send this to avoid_obs to clear the costmap at cone
  nav_state_pub_ = nh_.advertise<std_msgs::Int16>("nav_state",1);
  known_obs_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("known_obstacle",1);
  hill_wp_pub_ = nh_.advertise<std_msgs::Int16>("hill_wp",1);
  valid_bump_pub_ = nh_.advertise<std_msgs::Int16>("valid_bump",1);

  //Topic you want to subscribe
  scan_sub_ = nh_.subscribe("scan", 50, &NavStates::scanCallback, this); //receive laser scan
  odom_sub_ = nh_.subscribe("odom", 10, &NavStates::odomCallback, this);
  clicked_goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &NavStates::clickedGoalCallback, this);
  cam_cone_pose_sub_ = nh_.subscribe("cam_cone_pose", 1, &NavStates::camConeCallback, this);
  obs_cone_sub_ = nh_.subscribe("obs_cone_pose", 1, &NavStates::obsConeCallback, this);
  bump_sub_ = nh_.subscribe("bump_switch",1, &NavStates::bumpCallback, this);
  path_sub_ = nh_.subscribe("path",5, &NavStates::pathCallback, this);
  map_to_odom_update_sub_ = nh_.subscribe("map_to_odom_update", 1, &NavStates::mapToOdomUpdateCallback, this);
  nh_p  = ros::NodeHandle("~");
  nh_p.param("plan_rate_hz", params.plan_rate, 10.0);
  nh_p.param("use_PotFields", params.use_PotFields, false);
  nh_p.param("valid_cone_to_wp_dist", params.valid_cone_to_wp_dist, 1.0);
  nh_p.param("close_cone_to_bot_dist", params.close_cone_to_bot_dist, 1.0);

  nh_p.param("near_path_dist", params.near_path_dist, 1.0);
  nh_p.param("valid_end_of_path_dist", params.valid_end_of_path_dist, 5.0);
  nh_p.param("desired_speed", params.desired_speed, 0.6);
  nh_p.param("slow_speed", params.slow_speed, 0.3);
  nh_p.param("max_omega", params.max_omega, 0.5);
  nh_p.param("max_fwd_heading_error_deg", params.max_fwd_heading_error_deg, 90.0);
  nh_p.param("search_time", params.search_time, 1.0);
  nh_p.param("search_omega", params.search_omega, 0.1);
  nh_p.param("reverse_time", params.reverse_time, 2.0);
  nh_p.param("cmd_control_ver", params.cmd_control_ver, 0);
  nh_p.param("scan_collision_db_limit", params.scan_collision_db_limit, 2);
  nh_p.param("scan_collision_range", params.scan_collision_range, 0.5);
  nh_p.param("cone_detect_db_limit", params.cone_detect_db_limit, 2);
  nh_p.param("cmd_speed_filter_factor", params.cmd_speed_filter_factor, 0.5);
  nh_p.param("report_bumped_obstacles", params.report_bumped_obstacles, false);
  nh_p.param("max_camera_search_time", params.max_camera_search_time, 7.0);
  nh_p.param("slow_approach_distance", params.slow_approach_distance, 1.0);
  nh_p.param("reverse_speed", params.reverse_speed, 0.8);
  nh_p.param("bump_db_limit", params.bump_db_limit, 2);
  nh_p.param("path_step_size", params.path_step_size, 3);
  nh_p.param("ramp_time", params.ramp_time, 2.0);

  nh_p.param("x_coords", x_coords, x_coords);
  nh_p.param("y_coords", y_coords, y_coords);
  nh_p.param("waypoint_types", waypoint_type_list, waypoint_type_list);
  nh_p.param("hill_waypoint_list", hill_wp_list, hill_wp_list);
  nh_p.param("waypoints_are_in_map_frame", waypoints_are_in_map_frame, true);
  nh_p.param("sim_mode", sim_mode, false);

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
  camera_cone_pose_in_map = map_goal_pose;

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
  m_first_search = true; // used to reset the m_init_search_time in search_in_place state
  map_goal_pose.header.stamp = ros::Time::now();
  m_current_waypoint_type = waypoint_type_list[m_index_wp];
  m_current_hill_type = hill_wp_list[m_index_wp];
  map_goal_pose.pose.position.x = m_waypoints[m_index_wp].x;
  map_goal_pose.pose.position.y = m_waypoints[m_index_wp].y;
  camera_cone_pose_in_map = map_goal_pose;
  ROS_INFO("Update Waypoint");
  ROS_INFO("map x,y = %0.1f, %0.1f",map_goal_pose.pose.position.x, map_goal_pose.pose.position.y);
  wp_static_map_goal_pub_.publish(map_goal_pose);
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
    if(m_current_waypoint_type == WP_TYPE_CONE)
    {
      camera_cone_pose = odom_goal_pose;
      wp_cone_pub_.publish(camera_cone_pose); // publish this once to avoid obs so it will clear the costmap at the initial goal, until it finds an updated goal from the camera
    }
    ROS_INFO("wp_goal published");
    m_init_wp_published = true;
    m_path_received = false; //avoid updating waypoint again if Astar has not yet responded to the just updated waypoint
    m_obs_cone_received = false;
    m_cone_detect_db_count = 0;
  }
}

// Because can2ros now updates map_to_odom, update the goal in the new transformed odom frame
//  This is triggered by the mapToOdomUpdateCallback to topic "map_to_odom_update"
//  This will continue to be called until the getPoseinFrame is successful after a triggered refresh
void NavStates::refresh_odom_goal()
{
  //camera_cone_pose_in_map.header.stamp = ros::Time::now(); // Moved to the mapToOdomUpdateCallback
  geometry_msgs::PoseStamped odom_pose;
  odom_pose.header.frame_id = "odom";
  if(getPoseInFrame(camera_cone_pose_in_map, "odom", odom_pose))
  {
    odom_goal_pose = odom_pose;

    wp_goal_pub_.publish(odom_goal_pose);
    if(m_current_waypoint_type == WP_TYPE_CONE)
    {
      camera_cone_pose = odom_goal_pose;
      wp_cone_pub_.publish(camera_cone_pose); // publish this once to avoid obs so it will clear the costmap at the initial goal, until it finds an updated goal from the camera
    }
    m_odom_goal_refresh_needed = false;
  }
}

void NavStates::pathCallback(const nav_msgs::Path& path_in)
{
  // Verify m_path_received does not become true between update_waypoint() setting m_init_wp_published true and Astar publishing a new path to the old waypoint
  unsigned path_size = path_in.poses.size();
  if(path_size > 0)
  {
    // It is assumed that the next waypoint is far enough away such that the distance between the newly updated odom_goal_pose and a path to an old goal will be > valid_end_of_path_dist
    if(distance_between_poses(path_in.poses[path_size-1], odom_goal_pose) < params.valid_end_of_path_dist) //both poses should be in the odom frame
    {
      m_path = path_in;
      m_path_received = true;
      m_index_path = 0;
    }
  }

}

void NavStates::camConeCallback(const geometry_msgs::PoseStamped& cone_pose_in)
{
  // TODO: If state = search in place, set a boolean for search in place to move to state = STOP_AND_SEARCH
  //   we also want to do this from the track path state if we are near the end of path

  if(m_state != STATE_RETREAT_FROM_CONE && m_current_waypoint_type == WP_TYPE_CONE) //TODO: Verify we should ignore this for intermediate waypoints
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
      double dist = distance_between_poses(cone_in_map, map_goal_pose);
      ROS_INFO("camConeCallback, dist btwn cone and goal in map = %0.1f",dist);
      if(dist < params.valid_cone_to_wp_dist)
      {
        geometry_msgs::PoseStamped cone_in_odom;
        cone_in_odom.header.frame_id = "odom";
        if(getPoseInFrame(cone_pose_in, "odom", cone_in_odom) )
        {
          ++m_cone_detect_db_count;
          // TODO: Consider still checking bot to cone distance even though cone_finder also checks this
          if(m_cone_detect_db_count >= params.cone_detect_db_limit)
          {
            camera_cone_pose_in_map = cone_in_map;
            camera_cone_pose = cone_in_odom;
            wp_cone_pub_.publish(camera_cone_pose);
            m_cone_detect_db_count = params.cone_detect_db_limit;
            m_cone_detected = true;
            ROS_INFO("cone detected");
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

void NavStates::obsConeCallback(const geometry_msgs::PoseStamped& obs_cone_pose_in)
{
  obs_cone_pose = obs_cone_pose_in;
  m_obs_cone_received = true;
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
  m_bump_count += m_bump_switch;
  if(m_bump_switch == 0)
  {
    m_bump_count = 0;
  }

  if(m_bump_count >= params.bump_db_limit)
  {
    m_valid_bump = true;
  }
  else
  {
    m_valid_bump = false;
  }

  if(m_valid_bump && params.report_bumped_obstacles)
  {
    // TODO: Revisit the storage of known obs poses in Avoid Obs now that we will add potentially many known obstacles using the bumper
    //       Check for duplicate known obstacles in the deque, store them with low resolution so very close obstacles are considered duplicates
    //       Store the unique costmap index number of known obstacles and use that to check for duplicates
    // Known obstacle pose should be published in the odom frame for AvoidObs costmap
    if( (ros::Time::now()-m_bump_time).toSec() > params.reverse_time)
    {
      known_obs_pub_.publish(bot_pose);
      m_bump_time = ros::Time::now();
    }

  }
  std_msgs::Int16 valid_bump_msg;
  valid_bump_msg.data = m_valid_bump;
  valid_bump_pub_.publish(valid_bump_msg);
}

void NavStates::mapToOdomUpdateCallback(const std_msgs::Int16& msg)
{
  m_odom_goal_refresh_needed = true;
  camera_cone_pose_in_map.header.stamp = ros::Time::now();
}

double NavStates::get_plan_rate()
{
  return params.plan_rate;
}

void NavStates::odomCallback(const nav_msgs::Odometry& odom)
{
  bot_pose.header.stamp = odom.header.stamp;
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
  if(m_state == STATE_TOUCH_TARGET)
  {
	  return;
  }
  m_collision = false;
  unsigned close_count = 0;

  // Declare "collision" if multiple lasers see an obstacle within close range
  // TODO: Also calculate a close_obs_range to control m_speed in commandTo instead of using distance_between_poses
  //       Set a boolean here, m_close_to_obs = close_obs_range < params.slow_approach_distance
  double close_obs_range = DBL_MAX;
  for (int i = 0; i < scan.ranges.size();i++)
  {
    float range = scan.ranges[i];
    float angle  = scan.angle_min +(float(i) * scan.angle_increment);
    if(range < params.scan_collision_range)
    {
      ++close_count;
    }
    if(range < close_obs_range)
    {
      close_obs_range = range;
    }
  }

  m_close_to_obs = close_obs_range < params.slow_approach_distance;

  if(close_count > 0) // TODO: Parameter, how many lasers in one scan need to see an obstacle
  {
    ++m_scan_collision_db_count;
    if(m_scan_collision_db_count >= params.scan_collision_db_limit)
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
  // use m_close_to_obs from scanCallback
  if(m_state == STATE_TOUCH_TARGET &&
      ((distance_between_poses(bot_pose, odom_goal_pose) < params.slow_approach_distance) || m_close_to_obs) )
  {
    m_speed = params.slow_speed;
  }
  double des_yaw = getTargetHeading(goal);
  double heading_error = des_yaw - bot_yaw;
  if(heading_error >= M_PI)
  {
    heading_error -= 2*M_PI;
  }
  else if(heading_error < -M_PI)
  {
    heading_error += 2*M_PI;
  }

  if(params.cmd_control_ver == 0)
  {
    // ********** FROM DiffDriveController ********
    double ka= 1.0;
    double kb= 0.0;
    m_omega = ka*heading_error + kb*des_yaw;
    // **********************************************
  }
  else
  {
    m_omega = 0.5*(heading_error); // TODO: parameter
    if(fabs(m_omega) > 1.0) // slow for tight turns
    {
      m_omega = 1.0*m_omega/fabs(m_omega);
      m_speed = params.slow_speed;
    }
  }

  if(fabs(heading_error) > params.max_fwd_heading_error_deg*M_PI/180)
  {
    m_speed = -params.reverse_speed; //-m_speed
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

  if(m_valid_bump || m_collision)
  {
    m_state = STATE_RETREAT;
    m_speed = 0.0;
    m_omega = 0.0;
    return;
  }

  if(m_cone_detected && m_current_waypoint_type == WP_TYPE_CONE)
  {
	  if(distance_between_poses(bot_pose, odom_goal_pose) < params.close_cone_to_bot_dist) //TODO: Verify we should ignore m_cone_detected for intermediate waypoints
	  {
		m_state = STATE_TURN_TO_TARGET;
		return;
	  }
  }

  // end of path?, See pathCallback
  // Verify m_path_received does not become true between update_waypoint() setting m_init_wp_published true and Astar publishing a new path to the old waypoint
  if(m_path_received && m_index_path == m_path.poses.size()-1)
  {
    if(m_current_waypoint_type == WP_TYPE_CONE)
    {
      m_state = STATE_SEARCH_IN_PLACE;
      return;
    }
    else if(m_init_wp_published)
    {
      m_init_wp_published = false;
      update_waypoint();
    }
  }
  //check if close to next path point and update
  if(nearPathPoint())
  {
    m_index_path += params.path_step_size; //TODO: PARAMETER
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
  m_speed = -params.reverse_speed;
  m_omega = 0.0; //eventually retreat with m_omega = omega from path control
  if(get_time_in_state() > params.reverse_time)
  {
    m_speed = 0.0;
    m_state = STATE_TRACK_PATH;
  }
}
void NavStates::search_in_place()
{
  if(m_first_search)
  {
    m_init_search_time = ros::Time::now();
    m_first_search = false;
  }

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
    if( (ros::Time::now()-m_init_search_time).toSec() > params.max_camera_search_time)
    {
      if(m_obs_cone_received)
      {
        camera_cone_pose = obs_cone_pose;
      }
      else
      {
        camera_cone_pose = odom_goal_pose;
      }
      m_cone_detected = true; //could also not set this to still require camera cone detection
      update_target();
      //OR just update the goal for Astar to be the obs_cone_pose
      // update_target(obs_cone_pose); //still requires the camera to find the cone, but the path will be updated to go to obs_cone_pose
    }
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
  if(m_valid_bump) // || m_collision) m_collision means virtual bumper detected obstacle
  {
    // this causes update_waypoint() to be called again until it is successful
    m_init_wp_published = false;
    m_cone_detected = false;
    m_cone_detect_db_count = 0;

    update_waypoint();
    m_speed = 0.0;
    m_omega = 0.0;
    m_state = STATE_RETREAT_FROM_CONE;
    // Known obstacle pose should be published in the odom frame for AvoidObs costmap
    known_obs_pub_.publish(bot_pose);
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

  m_speed = -params.reverse_speed;
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

void NavStates::update_target(geometry_msgs::PoseStamped target_pose)
{
  wp_goal_pub_.publish(target_pose);
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

// Compute speed ramp parameters
float time_full_rev_to_fwd = params.ramp_time;	// Seconds
float delta_speed = (params.desired_speed + params.reverse_speed) / params.plan_rate / time_full_rev_to_fwd;
float speed = m_prev_speed;
//ROS_WARN("prev=%f, m_speed=%f, delta=%f", m_prev_speed, m_speed, delta_speed);


// Ramp to target speed
if (m_speed > m_prev_speed)
{
   speed = m_prev_speed + delta_speed;
   if (speed > m_speed)
     speed = m_speed;
}
else if (m_speed < m_prev_speed)
{
   speed = m_prev_speed - delta_speed;
   if (speed < m_speed)
     speed = m_speed;
}
//ROS_WARN("speed=%f", speed);

m_prev_speed = speed;
m_filt_speed = speed;
/*
  if(m_speed <= 0.0 && m_valid_bump)
  {
    m_filt_speed = m_speed;
  }
  else
  {
    double alpha = params.cmd_speed_filter_factor;
    m_filt_speed = m_filt_speed*alpha + m_speed*(1.0-alpha);
  }
*/

  if(fabs(m_filt_speed) > params.desired_speed)
  {
    m_filt_speed = 0.0;
  }
  if(fabs(m_omega) > params.max_omega)
  {
    m_omega = 0.0;
  }

  geometry_msgs::Twist cmd;
  cmd.linear.x = m_filt_speed;
  cmd.angular.z = m_omega;
  cmd_pub_.publish(cmd);

  m_nav_state_msg.data = m_state;
  nav_state_pub_.publish(m_nav_state_msg);

  m_hill_wp_msg.data = m_current_hill_type;
  hill_wp_pub_.publish(m_hill_wp_msg);

  if(m_odom_goal_refresh_needed)
  {
    refresh_odom_goal();
  }
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
