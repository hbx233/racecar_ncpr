#include <ros/ros.h>
#include "ncpr/common.h"
#include "ncpr/poly_trajectory.h"
#include <nav_msgs/Path.h>
using namespace ncpr;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "poly_trajectory");		
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Rate loop_rate(1);
  pub_ = nh_.advertise<nav_msgs::Path>("genearted_path" , 1);
  double total_time =10;
  double dt = 0.01;
  
  //test the genreated trajectory
  using TrajType = PolyTrajectory<double,2,3>;
  TrajType poly_traj;
  TrajType::OutputType start(0,5);
  TrajType::OutputType start_vel(1,0);
  TrajType::OutputType goal(5,2.5);
  TrajType::OutputType goal_vel(0,0);
  
  poly_traj.fitPolyTrajectory(start, goal, start_vel, goal_vel, total_time);
  
  // publish the path
  nav_msgs::Path path;
  
  path = poly_traj.generatePath(dt);
  
  while(1){
    pub_.publish(path);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}