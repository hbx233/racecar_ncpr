#include "ncpr/common.h"
#include "ncpr/local_planner.h"
#include "ncpr/poly_trajectory.h"
#include "ncpr/trajectory_tracker.h"

using namespace ncpr;
int main(int argc, char** argv){
	ros::init(argc, argv, "local_planner_test");
	ros::NodeHandle nh("~");
	ros::Rate loop_rate(20);
    LocalPlanner local_planner(nh);
  int spin_time = 0;
  while(1){
  	//cout<<"Ros Spin"<<endl;
    ros::spinOnce();
    loop_rate.sleep();
    //spin_time++;
    //cout<<"Spin Time: "<<spin_time<<endl;
  }
  
  return 0;
}