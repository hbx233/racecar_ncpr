#include <ros/ros.h>
#include "ncpr/common.h"
#include "ncpr/poly_trajectory.h"
#include "ncpr/trajectory_tracker.h"
using namespace ncpr;

int main(int argc, char** argv){
  //create a trajectory
  using TrajType = PolyTrajectory<double,2,3>;
  TrajType::Ptr poly_traj = std::make_shared<TrajType>();
  TrajType::OutputType start(-5,-3);
  TrajType::OutputType start_vel(0.877582561890373,0.479425538604203);
  TrajType::OutputType goal(0,0);
  TrajType::OutputType goal_vel(0.540302305868140,0.841470984807897);
  double total_time = 10;
  
  //create a trajectory tracker 
  TrajectoryTracker::Ptr traj_tracker = std::make_shared<TrajectoryTracker>(poly_traj);
  
  //first fit a trajectory 
  poly_traj->fitPolyTrajectory(start, goal, start_vel, goal_vel, total_time);
  cout<<poly_traj->getCoefficient()<<endl;

  TrajType::OutputType real_pos(-4.750000000000000,-2.750000000000000);
  TrajType::OutputType real_vel(0.877582561890373,0.479425538604203);
  cout<<"Input position and velocity"<<endl;
  double real_time=0;
  //Track the trajectory for one step 
  Vector3d control = traj_tracker->computeTrackingControl(real_pos, real_vel, real_time);
  cout<<control<<endl;
}