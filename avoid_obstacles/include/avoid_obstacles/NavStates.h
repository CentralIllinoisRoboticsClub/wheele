// Copyright 2019 coderkarl. Subject to the BSD license.

#ifndef NavStates_H
#define NavStates_H

//ROS Includes
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int16.h>
#include <string>
//#include <std_msgs/String.h>

class NavStates
{
public:
  NavStates();
  ~NavStates();
  //returns parameter rate_ in hz
  // used to define ros::Rate
  double get_plan_rate();
  void update_states();

private:
  void scanCallback(const sensor_msgs::LaserScan& scan);
  void odomCallback(const nav_msgs::Odometry& odom);
  void clickedGoalCallback(const geometry_msgs::PoseStamped& data);
  void camConeCallback(const geometry_msgs::PoseStamped& cone_pose_in);
  void obsConeCallback(const geometry_msgs::PoseStamped& obs_cone_pose_in);
  void bumpCallback(const std_msgs::Int16& data);
  void pathCallback(const nav_msgs::Path& path_in);
  void mapToOdomUpdateCallback(const std_msgs::Int16& data);

  void update_plan();
  bool check_for_cone_obstacle();
  void update_waypoint();
  void refresh_odom_goal();

  double distance_between_poses(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2);
  bool getPoseInFrame(const geometry_msgs::PoseStamped& pose_in, std::string target_frame,
      geometry_msgs::PoseStamped& pose_out);
  double getTargetHeading(const geometry_msgs::PoseStamped& goal);

  bool nearPathPoint();
  void commandTo(const geometry_msgs::PoseStamped& goal);
  void state_init();
  double get_time_in_state();

  // A function for each state in the state machine
  void track_path();
  void retreat();
  void search_in_place();
  void turn_to_target();
  void touch_target();
  void retreat_from_cone();
  void update_target();
  void update_target(geometry_msgs::PoseStamped target_pose);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_p;
  ros::Publisher cmd_pub_, wp_goal_pub_, wp_cone_pub_, wp_static_map_goal_pub_;
  ros::Publisher nav_state_pub_;
  ros::Publisher known_obs_pub_;
  ros::Publisher hill_wp_pub_;
  ros::Publisher valid_bump_pub_;

  ros::Subscriber scan_sub_, odom_sub_, clicked_goal_sub_, path_sub_;
  ros::Subscriber cam_cone_pose_sub_, laser_cone_pose_sub_, bump_sub_;
  ros::Subscriber obs_cone_sub_;
  ros::Subscriber map_to_odom_update_sub_;

  geometry_msgs::PoseStamped bot_pose, map_goal_pose, odom_goal_pose;
  geometry_msgs::PoseStamped camera_cone_pose, obs_cone_pose, camera_cone_pose_in_map;
  float bot_yaw;

  tf::TransformListener listener;
  nav_msgs::Path m_path;
  std_msgs::Int16 m_nav_state_msg, m_hill_wp_msg;

  std::vector<geometry_msgs::Point> m_waypoints;
  int m_current_waypoint_type;
  int m_current_hill_type;

  bool m_collision, m_cone_detected, m_odom_received, m_path_received, m_obs_cone_received;
  bool m_init_wp_published, m_odom_goal_refresh_needed;
  bool m_first_search;
  bool m_valid_bump;
  int m_bump_switch;
  int m_bump_count;
  unsigned m_num_waypoints, m_index_wp;
  int m_state;
  unsigned m_index_path;
  double m_speed, m_omega, m_filt_speed;
  unsigned m_scan_collision_db_count;
  unsigned m_cone_detect_db_count;

  bool m_close_to_obs;

  ros::Time state_start_time;
  ros::Time m_bump_time;
  ros::Time m_init_search_time;

  //parameters
  struct Parameters
  {
    double plan_rate; //Default 10 Hz, how often we use potential fields to update cmd_vel
    bool use_PotFields;
    double valid_cone_to_wp_dist;
    double near_path_dist;
    double valid_end_of_path_dist;
    double desired_speed;
    double slow_speed;
    double max_omega;
    double max_fwd_heading_error_deg;
    double search_time;
    double search_omega;
    double reverse_time;
    int cmd_control_ver;
    int scan_collision_db_limit;
    double scan_collision_range;
    int cone_detect_db_limit;
    double cmd_speed_filter_factor;
    bool report_bumped_obstacles;
    double max_camera_search_time;
    double slow_approach_distance;
    double reverse_speed;
    int bump_db_limit;
    int path_step_size;
    //int min_new_path_size;
  }params;

  std::vector<double> x_coords;
  std::vector<double> y_coords;
  std::vector<int> waypoint_type_list;
  std::vector<int> hill_wp_list;
  bool waypoints_are_in_map_frame;
  bool sim_mode;

};

#endif
