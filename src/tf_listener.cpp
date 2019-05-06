#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

void ackermannCallback(const ackermann_msgs::AckermannDriveStamped& msg)
{
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_listener");
  ros::NodeHandle nh_;
  ros::Subscriber sub = nh_.subscribe("/vesc/i", 1000, odomCallback);
  ros::spin();

  return 0;
}
