#include "ncpr/common.h"
#include "ncpr/poly_trajectory.h"
#include "ncpr/trajectory_tracker.h"
#include "ncpr/local_planner.h"

namespace ncpr {
LocalPlanner::LocalPlanner(const ros::NodeHandle& nh)
{
  // Get parameters
  nh_.getParam("car/length", car_length_);
  nh_.getParam("global_path", topic_globalpath_);
  nh_.getParam("trajectory/lookahead", look_ahead_);
  nh_.getParam("trajectory/local_path", topic_localpath_);
  nh_.getParam("trajectory/total_time", total_time_);
  nh_.getParam("trajectory/reference_velocity", reference_velocity_);
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
  local_traj_tracker_ptr_ = std::make_shared<TrajectoryTracker>(local_trajectory_ptr_, car_length_, kp_, kd_);
  
  state_ = State::WaitForGlobal;
}

double pose2PoseDist(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2){
  return (std::pow(pose1.position.x-pose2.position.x,2) + std::pow(pose1.position.y - pose2.position.y,2));
}

int LocalPlanner::findNearestInGlobalPath(const geometry_msgs::Pose& pose)
{
  int min_idx = 0;
  double min_dist = std::numeric_limits< double >::max();
  for(int i=0; i<global_path_.poses.size(); i++){
    double dist = pose2PoseDist(pose, global_path_.poses[i].pose);
    if(dist < min_dist){
      min_idx = i;
      min_dist = dist;
    }
  }
  return min_idx;
}

PolyTrajectory< double, OUTPUT, BASIS >::OutputType LocalPlanner::poseToOutputVector(const geometry_msgs::Pose& pose)
{
  PolyTrajectory<double, OUTPUT, BASIS>::OutputType vec;
  vec<<pose.position.x,pose.position.y;
  return vec;
}

PolyTrajectory< double, OUTPUT, BASIS >::OutputType LocalPlanner::twistToOutputVector(const geometry_msgs::Twist& twist)
{
  PolyTrajectory<double, OUTPUT, BASIS>::OutputType vec;
  vec<<twist.linear.x, twist.linear.y;
  return vec;
}

PolyTrajectory< double, OUTPUT, BASIS >::OutputType LocalPlanner::calculateVelocity(const int& index, const double& vel_magnitude)
{
  PolyTrajectory<double, OUTPUT, BASIS>::OutputType vel;
  if(index >= global_path_.poses.size()-1){
    //set final goal velocity to be zero 
    vel = PolyTrajectory<double, OUTPUT, BASIS>::OutputType::Zero();
  } else{
    //use the direction of pre neighbor to next neighbor as direction of velocity 
    geometry_msgs::Pose prev_pose = global_path_.poses[index-1<0? 0:index-1].pose;
    geometry_msgs::Pose next_pose = global_path_.poses[index+1].pose;
    double dist = std::sqrt(pose2PoseDist(prev_pose, next_pose));
    cout<<"[Debug] Planner, calculate velocity"<<endl;
    cout<<"[Debug] Dist: "<<dist<<endl;
    cout<<"[Debug] X: "<< next_pose.position.x - prev_pose.position.x<<endl;
    cout<<"[Debug] Y: "<< next_pose.position.y - prev_pose.position.y<<endl;
    
    //calculate direction
    vel<<(next_pose.position.x - prev_pose.position.x)/dist,(next_pose.position.y - prev_pose.position.y)/dist;
    cout<<"[Debug] Vel: "<<vel<<endl;
    //mult with magnitude
    vel*=vel_magnitude;
  }
  return vel;
}


void LocalPlanner::calculateStartAndGoal(const int& start_idx, const double& start_vel_mag){
  //calculate the start
  local_start_ = poseToOutputVector(global_path_.poses[start_idx].pose);
  local_start_vel_ = calculateVelocity(start_idx, 1)*start_vel_mag;
  local_start_idx_ = start_idx;
  //look ahead to set local goal in global path
  local_goal_idx_ = start_idx + look_ahead_ >= global_path_.poses.size() ? global_path_.poses.size()-1 : start_idx + look_ahead_;
  local_goal_ = poseToOutputVector(global_path_.poses[local_goal_idx_].pose);
  local_goal_vel_ = calculateVelocity(local_goal_idx_, reference_velocity_);
}

void LocalPlanner::globalPath_callback(const nav_msgs::Path& msg)
{
  ROS_INFO("[Debug] GlobalPath Callback \n");
  if(state_ == State::WaitForGlobal &&  msg.poses.size() != 0){
    ROS_INFO("[Local Planner] Receving Global path...");
    global_path_ = msg;
    state_ = State::Running;
    ROS_INFO("[Local Planner] Received Global path, begin local planning");
  }
  else{
    ROS_ERROR("Empty Global path !");
  }
}

void LocalPlanner::pose_tracking_callback(const nav_msgs::Odometry& msg){
  ROS_INFO("[Debug] Tracking Callback \n");
  if(state_ == State::Running && local_trajectory_ptr_->valid() == true){
    //get current input, time
    //TODO: Use time stamp  or get a new time 
    ROS_INFO("[Local Planner] Start Local Tracking \n");
    double curr_time = ros::Time::now().toSec();
    PolyTrajectory<double, OUTPUT, BASIS>::OutputType pos = poseToOutputVector(msg.pose.pose);
    PolyTrajectory<double, OUTPUT, BASIS>::OutputType vel = twistToOutputVector(msg.twist.twist);
    //TODO: Need to pass in a theta or use velocity to get theta 
    double theta = 2 * std::asin(msg.pose.pose.orientation.z);
    Vector2d control = local_traj_tracker_ptr_->computeTrackingControl(pos,vel,theta,curr_time);
    //Publish control, ackermann_msgs
    double steering_angle;
    if(control(1)>0.34){
      steering_angle = 0.34;
    } else if(control(1)<-0.34){
      steering_angle = -0.34;
    } else{
      steering_angle = control(1);
    }
    double speed;
    if(control(0) > 0){
      speed = reference_velocity_;
    } else if(control(0) <0){
      speed = -reference_velocity_;
    } else{
      speed = std::sqrt(std::pow(vel(0),2)+std::pow(vel(1),2));
    }
    ackermann_msgs::AckermannDriveStamped ackermann_control;
    ackermann_control.header.stamp = ros::Time::now();
    ackermann_control.drive.speed = speed;
    ackermann_control.drive.acceleration = control(0);
    ackermann_control.drive.jerk = 0.0;
    ackermann_control.drive.steering_angle = steering_angle;
    ackermann_control.drive.steering_angle_velocity = 0.0;
    ROS_INFO("[Local Planner] Publishing Control \n");
    pub_control_.publish(ackermann_control); 
    ROS_INFO("[Local Planner] Published Control \n");
  } else if(state_ == State::Reached){
    ROS_INFO("[Local Planner] Global Path reached");
    ackermann_msgs::AckermannDriveStamped ackermann_control;
    ackermann_control.drive.speed = 0;
    ackermann_control.drive.acceleration = 0;
    ackermann_control.drive.jerk = 0.0;
    ackermann_control.drive.steering_angle = 0;
    ackermann_control.drive.steering_angle_velocity = 0.0;
    pub_control_.publish(ackermann_control); 
  }
}

void LocalPlanner::pose_trajectory_callback(const nav_msgs::Odometry& msg)
{
  if(state_==State::Running){
    //Running
    geometry_msgs::Pose curr_pose = msg.pose.pose;
    double curr_vel_mag = std::sqrt(std::pow(msg.twist.twist.linear.x,2)+std::pow(msg.twist.twist.linear.y,2));
    nav_msgs::Path local_path;
    //Initial Local Planning
    //TODO: Switch and replan when 
    //1. reached local goal in certian threshold 
    //2. local trajectory tracking time out(exceeded the seted total time)
    cout<<"[Local Planner] Current Trajectory ID: "<<local_trajectory_ptr_->getID();
    if(local_trajectory_ptr_->getID() == 0){
      //start from start index in global path 
      ROS_INFO("[Local Planner] Start Local Planner from initial pose in Global Path \n");
      local_start_idx_ = 0;
      calculateStartAndGoal(local_start_idx_, reference_velocity_);
      local_trajectory_ptr_->fitPolyTrajectory(local_start_, local_goal_, local_start_vel_, local_goal_vel_, total_time_);
      local_path = local_trajectory_ptr_->generatePath(0.01);
      ROS_INFO("[Local Planner] Publishing Local Trajectory \n");
      pub_localpath_.publish(local_path);
      ROS_INFO("[Local Planner] Published Local Trajectory \n");
    } else{
      //check if reached local goal 
      if(std::sqrt(pose2PoseDist(curr_pose, global_path_.poses[local_goal_idx_].pose))<goal_threshold_){
	      //switch goal to start 
	      ROS_INFO("[Local Planner] Local Goal Reached, Switch local goal to next local start \n");
        if(local_goal_idx_ == global_path_.poses.size()-1){
          state_ = State::Reached;
        }
	      local_start_idx_ = local_goal_idx_;
	      calculateStartAndGoal(local_start_idx_, curr_vel_mag);
        local_trajectory_ptr_ -> fitPolyTrajectory(local_start_, local_goal_, local_start_vel_, local_goal_vel_,total_time_);
        local_path = local_trajectory_ptr_->generatePath(0.01);
        pub_localpath_.publish(local_path);
      } else{
	      //check time out,
	      if(local_traj_tracker_ptr_->getTrackingElapsedTime() > local_trajectory_ptr_->getTotalTime()){
	        //replan
	        ROS_INFO("[Local Planner] Local tracking time out, Replanning \n");
	        int nearest_idx = findNearestInGlobalPath(curr_pose);
	        calculateStartAndGoal(nearest_idx, curr_vel_mag);
          local_trajectory_ptr_ -> fitPolyTrajectory(local_start_, local_goal_, local_start_vel_, local_goal_vel_,total_time_);
	        local_path = local_trajectory_ptr_->generatePath(0.01);
          pub_localpath_.publish(local_path);
        } else{
	        //maintain current tracking
	        ROS_INFO("[Local Planner] Still tracking \n");
	      }
      }
    }
  }
}

}
