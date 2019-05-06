#ifndef LOCAL_PLANNER_H_
#define LOCAL_PLANNER_H_

#include "ncpr/common.h"
#include "ncpr/poly_trajectory.h"
#include "ncpr/trajectory_tracker.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

//Order of Polynomial basis function 
#define BASIS 3
//Output dimension 
#define OUTPUT 2

namespace ncpr{
  class LocalPlanner{
  public:
    /*!
     * @breif Constructors that do initialization and connect all publishers and subscribers
     */
    explicit LocalPlanner(const ros::NodeHandle& nh);
    
    enum class State {Initializing,WaitForGlobal, Running, Reached};
    State state_{State::Initializing};
  public:
    void Run();
  private: 
    //subscribers and Callback functions 
    //Global path subscriber and callback functions, receive global path and cache it 
    string topic_globalpath_;
    ros::Subscriber sub_globalpath_;
    /*!
     * @brief Callback functions for global path, will transfer the state of local planner to Running state
     * @param msg global path message 
     */
    void globalPath_callback(const nav_msgs::Path& msg);
    //Current pose subscriber and callback functions 
    //TODO: Need to get current position and velocity, How to get velocity 
    string topic_odom_;
    ros::Subscriber sub_pose_trajectory_;
    /*!
     * @brief Pose Callback function for trajectory generation
     * 
     */
    void pose_trajectory_callback(const nav_msgs::Odometry& msg);
    int look_ahead_;
    double reference_velocity_;
    double total_time_;

    //publisher to publish global path, only receive once to visualize
    string topic_globalpath_rviz_;
    ros::Publisher pub_global_path_rviz_;
    
    //publisher to publish local path, only used to visualize 
    string topic_localpath_;
    ros::Publisher pub_localpath_;

    
  private:
    //Tracking and Control part 
    //Tracking parameter 
    double car_length_; //length of car
    double kp_; //p term
    double kd_; //d term
    double goal_threshold_; //threshold for checking if reached the goal
    
    //tracking subscriber 
    ros::Subscriber sub_pose_tracking_;
    /*!
     * @brief Pose Callback function for trajectory tracking 
     */
    void pose_tracking_callback(const nav_msgs::Odometry& msg);
    
    //Control Publisher
    string topic_control_; //control topic name 
    ros::Publisher pub_control_; //control publisher, ackermann_msgs::AckermannDriveStamped
  private:
    //start and end 
    /*!
     * @brief Helper function that convert pose to output vector that can be used to fit trajectory 
     * @param pose From global path or subscribed current pose from gazebo.
     */
    PolyTrajectory<double,OUTPUT,BASIS>::OutputType poseToOutputVector(const geometry_msgs::Pose& pose);
    /*!
     * @brief Helper function that convert twist to output vector
     * @param twist twist value
     */
    PolyTrajectory<double,OUTPUT,BASIS>::OutputType twistToOutputVector(const geometry_msgs::Twist& twist);
    /*!
     * @brief Calculate desired velocity given index of pose in global path
     * @param index Index of given pose in global path
     * @return Desired velocity 
     */
    PolyTrajectory<double,OUTPUT,BASIS>::OutputType calculateVelocity(const int& index, const double& vel_magnitude);
    
    /*!
     * @brief Find nearest pose in the global path
     */
    int findNearestInGlobalPath(const geometry_msgs::Pose& pose);
    
    /*!
     * @brief Calculate local trajectory's start and goal position and velocity, 
     *        Set the calculated value to class member fields 
     */
    void calculateStartAndGoal(const int& start_idx, const double& start_vel_mag);
    //Trajectory Generation Parameter 
    PolyTrajectory<double,OUTPUT,BASIS>::OutputType local_start_;
    int local_start_idx_;
    PolyTrajectory<double,OUTPUT,BASIS>::OutputType local_goal_;
    int local_goal_idx_;
    PolyTrajectory<double,OUTPUT,BASIS>::OutputType local_start_vel_;
    PolyTrajectory<double,OUTPUT,BASIS>::OutputType local_goal_vel_;
  private: 
    ros::NodeHandle nh_;
    nav_msgs::Path global_path_;
    nav_msgs::Path local_path_;
    
    TrajectoryTracker::Ptr local_traj_tracker_ptr_;
    PolyTrajectory<double,OUTPUT,BASIS>::Ptr local_trajectory_ptr_;
  };
}


#endif