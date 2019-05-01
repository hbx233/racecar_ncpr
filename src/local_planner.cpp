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
  local_traj_tracker_ptr_ = std::make_shared<TrajectoryTracker>(local_traj_tracker_ptr_);
}


  
}
