#include "ncpr/common.h"
#include "ncpr/poly_trajectory.h"
#include "ncpr/trajectory_tracker.h"
#include "ncpr/local_planner.h"

namespace ncpr {
LocalPlanner::LocalPlanner(const ros::NodeHandle& nh)
{
  // Get parameters
  nh_.getParam("car/length");

  // Connect all the subscribers and publishers
  sub_pose_tracking_ = nh_.subscribe(topic_name_[tracking]);

  // Create PolyTrajectory Instance
  local_trajectory_ptr_ = std::make_shared<PolyTrajectory<double,OUTPUT,BASIS>>();

  // Create TrajectoryTracker
  local_traj_tracker_ptr_ = std::make_shared<TrajectoryTracker>(local_trajectory_ptr_);
}

void LocalPlanner::run()
{

}

void LocalPlanner::globalPath_callback(const nav_msgs::Path& msg)
{
  if(msg.path.size() != 0){
    ROS_INFO("receving Global path...");
    global_path_ = msg;
    state_ = State::Running;
  }
  else{
    ROS_ERROR("Empty Global path !");
  }

}

void LocalPlanner::pose_trajectory_callback(const nav_msgs::Odometry& msg)
{
  if(state_==State::Running){
    //Running, begin local planner
    if(local_trajectory_ptr_->getID() == 0){
      start_ =
      goal_ =
      start_vel_ =
      goal_vel_ =
      total_time_ =
      local_trajectory_ptr_->fitPolyTrajectory(start_, g)
    } else{
      //if reached global goal, transit state
      if(){
        state_ = State::Reached;
      } else{
        //if reached local goal
        if(){
          //choose start and goal,

          //fit new local trajectory
          local_trajectory_ptr_->fitPolyTrajectory(...);
        }
      }
    }
  }
}

void LocalPlanner::pose_tracking_callback(const nav_msgs::Odometry& msg){
  if(state_ == State::Running){
    //get current input, time
    Vector3d control = local_traj_tracker_ptr_->computeTrackingControl();
    //Publish control, ackermann_msgs
  }
}

void LocalPlanner::publishControlToAckermann(Eigen::Vector2d control)

}
