#ifndef LOCAL_PLANNER_H_
#define LOCAL_PLANNER_H_

#include "ncpr/common.h"
#include "ncpr/poly_trajectory.h"
#include "ncpr/trajectory_tracker.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

//Order of Polynomial basis function 
#define BASIS 3
//Output dimension 
#define OUTPUT 2

namespace ncpr{
  class LocalPlanner{
  public:
    explicit LocalPlanner(const ros::NodeHandle& nh);
    
    
    enum class State {WaitForGlobal, Tracking, Reached};
    State state_{State::WaitForGlobal};
  public:
    void Run(){
      switch(state_){
	case State::WaitForGlobal:
	  
      }
    }
  private: 
    //subscribers and Callback functions 
    //Global path subscriber and callback functions, receive global path and cache it 
    ros::Subscriber sub_globalpath_;
    void globalPath_callback(const nav_msgs::Path& msg);
    //Current pose subscriber and callback functions 
    //TODO: Need to get current position and velocity, How to get velocity 
    ros::Subscriber sub_pose_;
    void pose_callback(const geometry_msgs::Pose& msg);
  private:
    //Publisher and their publish function 
    //Control publisher 
    ros::Publisher pub_control;
    /*!
     * @brief convert control to ackermann_msgs and publish it 
     * @param control the high level control signals that pass to low level controller 
     */
    void publishControl(Eigen::Vector2d control);
  private:
    //start and end 
    /*!
     * @brief Helper function that convert pose to output vector that can be used to fit trajectory 
     * @param pose From global path or subscribed current pose from gazebo.
     */
    PolyTrajectory<double,OUTPUT,BASIS>::OutputType poseToOutputVector(const geometry_msgs::Pose pose);
    //TODO: Write a tf listener node which listen to tf and publish current pose 
    geometry_msgs::Pose curr_pose_;
    PolyTrajectory<double,OUTPUT,BASIS>::OutputType start_;
    PolyTrajectory<double,OUTPUT,BASIS>::OutputType goal_;
    PolyTrajectory<double,OUTPUT,BASIS>::OutputType start_vel_;
    PolyTrajectory<double,OUTPUT,BASIS>::OutputType goal_vel_;
  private: 
    ros::NodeHandle nh_;
    nav_msgs::Path global_path_;
    nav_msgs::Path local_path_;
    PolyTrajectory<double,OUTPUT,BASIS>::Ptr local_trajectory_ptr_;
    
  };
}


#endif