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
  void bumpCallback(const std_msgs::Int16& data);
  void pathCallback(const nav_msgs::Path& path_in);

  void update_plan();
  bool check_for_cone_obstacle();
  void update_waypoint();
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

  ros::NodeHandle nh_;
  ros::NodeHandle nh_p;
  ros::Publisher cmd_pub_, wp_goal_pub_, wp_cone_pub_;
  ros::Publisher obs_cone_pub_;
  ros::Publisher nav_state_pub_;
  ros::Publisher known_obs_pub_;

  ros::Subscriber scan_sub_, odom_sub_, clicked_goal_sub_, path_sub_;
  ros::Subscriber cam_cone_pose_sub_, laser_cone_pose_sub_, bump_sub_;

  geometry_msgs::PoseStamped bot_pose, map_goal_pose, odom_goal_pose;
  geometry_msgs::PoseStamped camera_cone_pose, obs_cone_pose;
  float bot_yaw;

  tf::TransformListener listener;
  nav_msgs::Path m_path;
  std_msgs::Int16 m_nav_state_msg;

  std::vector<geometry_msgs::Point> m_waypoints;

  bool m_collision, m_cone_detected, m_odom_received, m_path_received;
  bool m_init_wp_published;
  int m_bump_switch;
  unsigned m_num_waypoints, m_index_wp;
  int m_state;
  unsigned m_index_path;
  double m_speed, m_omega, m_filt_speed;
  unsigned m_scan_collision_db_count;
  unsigned m_cone_detect_db_count;

  ros::Time state_start_time;

  //parameters
  struct Parameters
  {
    double plan_rate; //Default 10 Hz, how often we use potential fields to update cmd_vel
    bool use_PotFields;
    double close_cone_to_bot_dist;
    double valid_cone_to_wp_dist;
    double near_path_dist;
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
  }params;

  std::vector<double> x_coords;
  std::vector<double> y_coords;
  bool waypoints_are_in_map_frame;

};

#endif
