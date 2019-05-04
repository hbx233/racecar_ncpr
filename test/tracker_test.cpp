#include <ros/ros.h>
#include "ncpr/common.h"
#include "ncpr/poly_trajectory.h"
#include "ncpr/trajectory_tracker.h"
using namespace ncpr;

int main(int argc, char** argv){
  //create a trajectory
  using TrajType = PolyTrajectory<double,2,3>;
  TrajType::Ptr poly_traj = std::make_shared<TrajType>();
  TrajType::OutputType start(0,5);
  TrajType::OutputType start_vel(1,0);
  TrajType::OutputType goal(5,2.5);
  TrajType::OutputType goal_vel(0,0);
  double total_time = 10;
  
  //create a trajectory tracker 
  TrajectoryTracker::Ptr traj_tracker = std::make_shared<TrajectoryTracker>(poly_traj);
  
  //first fit a trajectory 
  poly_traj->fitPolyTrajectory(start, goal, start_vel, goal_vel, total_time);
  
  TrajType::OutputType real;
  TrajType::OutputType real_vel;
  double real_time;
  //Track the trajectory for one step 
  Vector3d control = traj_tracker->computeTrackingControl(real, real_vel, real_time);
  cout<<control<<endl;
}