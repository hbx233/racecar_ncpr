#include "ncpr/common.h"
#include "ncpr/poly_trajectory.h"
#include "ncpr/trajectory_tracker.h"
#include "ncpr/local_planner.h"

namespace ncpr {
LocalPlanner::LocalPlanner(const ros::NodeHandle& nh)
{
  // Get parameters
  nh_.getParam("car/length");
  nh_.getParam("global_path", topic_globalpath_);
  nh_.getParam("trajectory/lookahead", look_ahead_);
  nh_.getParam("trajectory/local_path", topic_localpath_);
  nh_.getParam("tracking/odom", topic_odom_);
  nh_.getParam("tracking/control", topic_control_);
  nh_.getParam("tracking/kp", kp_);
  nh_.getParam("tracking/kd", kd_);
  nh_.getParam("tracking/goal_threshold", goal_threshold_);
  // Connect all the subscribers and publishers
  //subscribers 
  sub_globalpath_ = nh_.subscribe(topic_globalpath_, 10, &LocalPlanner::globalPath_callback, this);
  sub_pose_trajectory_ = nh_.subscribe(topic_odom_, 10, &LocalPlanner::pose_trajectory_callback, this);
  sub_pose_tracking_ = nh_.subscribe(topic_odom_, 10, &LocalPlanner::pose_tracking_callback, this);
  
  //publishers
  pub_localpath_ = nh_.advertise<nav_msgs::Path>(topic_localpath_, 10, true);
  pub_control_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(topic_control_, 10, true);
  
  // Create PolyTrajectory Instance
  local_trajectory_ptr_ = std::make_shared<PolyTrajectory<double,OUTPUT,BASIS>>();

  // Create TrajectoryTracker
  local_traj_tracker_ptr_ = std::make_shared<TrajectoryTracker>(local_traj_tracker_ptr_);
  
  state_ = State::WaitForGlobal;
}

void LocalPlanner::run()
{

}

void LocalPlanner::globalPath_callback(const nav_msgs::Path& msg)
{
  if(msg.poses.size() != 0){
    ROS_INFO("receving Global path...");
    global_path_ = msg;
    state_ = State::Running;
  }
  else{
    ROS_ERROR("Empty Global path !");
  }

}

void LocalPlanner::pose_trajectory_callback(const geometry_msgs::Pose& msg)
{
  if(state_==State::Running){
    //Running
  }
}



}
