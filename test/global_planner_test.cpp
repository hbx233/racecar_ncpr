#include <ros/ros.h>
#include "ncpr/common.h"
#include "ncpr/poly_trajectory.h"
#include <nav_msgs/Path.h>
using namespace ncpr;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_planner");		
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Rate loop_rate(1);
  pub_ = nh_.advertise<nav_msgs::Path>("global_path" , 1);
  double total_time =10;
  double dt = 0.01;
  

  // publish the path
  nav_msgs::Path path;
  geometry_msgs::Pose pose;
  geometry_msgs::PoseStamped pose_stamped;
  //TODO: Need Header
  std_msgs::Header header_msg;
  header_msg.stamp = ros::Time::now();
  header_msg.frame_id = "map";
  path.header = header_msg;

  for(int i = 0; i < 4; i++)
  {
    pose.position.y = 1*i;
    pose.position.x = std::pow(i,2) + 2*i;
    pose_stamped.pose = pose;
    path.poses.push_back(pose_stamped);
  }

  while(1){
    pub_.publish(path);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}