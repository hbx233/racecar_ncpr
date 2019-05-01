#include "ncpr/common.h"
#include "ncpr/poly_trajectory.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
using namespace ncpr;
int main(){
  //Test Case 1
  using TrajType = PolyTrajectory<double,2,3>;
  TrajType poly_traj;
  TrajType::OutputType start(0,5);
  TrajType::OutputType start_vel(1,0);
  TrajType::OutputType goal(5,2.5);
  TrajType::OutputType goal_vel(0,0);
  double total_time = 10;
  poly_traj.fitPolyTrajectory(start, goal, start_vel, goal_vel, total_time);
  
  cout<<poly_traj.getCoefficient()<<endl;
}