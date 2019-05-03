#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("receving Odometry...");
  double state_x = msg.pose.pose.positon.x;
  double state_y = msg.pose.pose.positon.y;

  double vel_x = msg.twist.twist.linear.x;
  double vel_y =msg.twist.twist.linear.y;
  double w = msg.twist.twist.angular.z;

  double vel = sqrt(vel_x*vel_x + vel_y*vel_y);
  double theta = atan2(vel_y,vel_x);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_listener");
  ros::NodeHandle nh_;
  ros::Subscriber sub = nh_.subscribe("vesc/odom", 1000, odomCallback);
  ros::spin();

  return 0;
}
