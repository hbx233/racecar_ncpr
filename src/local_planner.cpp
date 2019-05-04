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
  if(state_ = State::Initializing &&  msg.poses.size() != 0){
    ROS_INFO("receving Global path...");
    global_path_ = msg;
    state_ = State::Running;
    ROS_INFO("Received Global path, begin local planning");
  }
  else{
    ROS_ERROR("Empty Global path !");
  }
}



double pose2PoseDist(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2){
  return (std::pow(pose1.position.x-pose2.position.x,2) - std::pow(pose1.position.y - pose2.position.y));
}

int LocalPlanner::findNearestInGlobalPath(const geometry_msgs::Pose& pose)
{
  int min_idx = 0;
  double min_dist = std::numeric_limits< double >::max();
  for(int i=0; i<global_path_.poses.size(); i++){
    double dist = pose2PoseDist(pose, global_path_.poses[i].pose);
    if(dist < min_dist){
      min_idx = i;
    }
  }
  return min_idx;
}

PolyTrajectory< double, OUTPUT, BASIS >::OutputType LocalPlanner::poseToOutputVector(const geometry_msgs::Pose& pose)
{
  PolyTrajectory<double, OUTPUT, BASIS>::OutputType vec;
  vec<<pose.position.x<<pose.position.y;
  return vec;
}

PolyTrajectory< double, OUTPUT, BASIS >::OutputType LocalPlanner::calculateVelocity(const int& index, const double& vel_magnitude)
{
  PolyTrajectory<double, OUTPUT, BASIS>::OutputType vel;
  if(index == global_path_.poses.size()-1){
    //set final goal velocity to be zero 
    vel = PolyTrajectory<double, OUTPUT, BASIS>::OutputType::Zero();
  } else{
    //use the direction of pre neighbor to next neighbor as direction of velocity 
    geometry_msgs::Pose prev_pose = global_path_.poses[i-1<0? 0:i-1].pose;
    geometry_msgs::Pose next_pose = global_path_.poses[i+1].pose;
    double dist = std::sqrt(pose2PoseDist(prev_pose, next_pose));
    //calculate direction
    vel<<(next_pose.position.x - prev_pose.position.x)/dist<<(next_pose.position.y - prev_pose.position.y)/dist;
    //mult with magnitude
    vel*=vel_magnitude_;
  }
  return vel;
}


void LocalPlanner::calculateStartAndGoal(const int& start_idx){
  //calculate the start
  local_start_ = poseToOutputVector(global_path_.poses[start_idx].pose);
  local_start_vel_ = calculateVelocity(start_idx, vel_magnitude_);
  //look ahead to set local goal in global path
  int local_goal_idx = start_idx + look_ahead_ >= global_path_.poses.size() ? global_path_.poses.size()-1 : nearest_idx + look_ahead_;
  local_goal_ = poseToOutputVector(global_path_.poses[local_goal_idx].pose);
  local_goal_vel_ = calculateVelocity(local_goal_idx, vel_magnitude_);
}

void LocalPlanner::pose_tracking_callback(const nav_msgs::Odometry& msg){
  if(state_ == State::Running){
    //get current input, time
    //TODO: Use time stamp  or get a new time 
    double curr_time = ros::Time::now().toSec();
    PolyTrajectory<double, OUTPUT, BASIS>::OutputType pos = poseToOutputVector(msg.pose.pose);
    //TODO: Need to pass in a theta or use velocity to get theta 
    Vector3d control = local_traj_tracker_ptr_->computeTrackingControl(msg.pose.pose,msg.twist.twist.linear,curr_time);
    //Publish control, ackermann_msgs
    ackermann_msgs::AckermannDriveStamped ackermann_control;
    ackermann_control.drive.speed = ;
    ackermann_control.drive.acceleration = ;
    ackermann_control.drive.jerk = ;
    ackermann_control.drive.steering_angle = ;
    ackermann_control.drive.steering_angle_velocity = ;
    pub_control_.publish(ackermann_control);
  }
}

void LocalPlanner::pose_trajectory_callback(const nav_msgs::Odometry& msg)
{
  if(state_==State::Running){
    //Running
    geometry_msgs::Pose curr_pose = msg.pose.pose;
    //Initial Local Planning
    //TODO: Switch and replan when 
    //1. reached local goal in certian threshold 
    //2. local trajectory tracking time out(exceeded the seted total time)
    if(local_trajectory_ptr_->getID() == 0){
      //start from start index in global path 
      calculateStartAndGoal(0);
      double total_time = 10;
      local_trajectory_ptr_->fitPolyTrajectory(local_start_, local_goal_, local_start_vel_, local_goal_vel_, total_time);
    } else{
      //check if reached local goal 
      if(std::sqrt(pose2PoseDist())<goal_threshold_){
	
      } else{
	//check time out,
	if(time out){
	  //replan
	} else{
	  //maintain current tracking 
	}
      }
    }
  }
}

}
