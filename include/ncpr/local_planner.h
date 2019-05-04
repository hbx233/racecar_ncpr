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
    void pose_trajectory_callback(const geometry_msgs::Pose& msg);
    
    //publisher to publish local path, only used to visualize 
    string topic_localpath_;
    ros::Publisher pub_localpath_;
    
    ros::Subscriber sub_pose_tracking_;
    /*!
     * @brief Pose Callback function for trajectory tracking 
     */
    void pose_tracking_callback(const geometry_msgs::Pose& msg);
  private:
    //Tracking and Control part 
    //Control publisher 
    double car_length_;
    double kp_;
    double kd_;
    double goal_threshold_;
    string topic_control_;
    ros::Publisher pub_control_;
    /*!
     * @brief convert control to ackermann_msgs and publish it 
     * @param control the high level control signals that pass to low level controller 
     */
    void publishControlToAckermann(Eigen::Vector3d control);
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
    void calculateStartAndGoal(const geometry_msgs::Pose& pose);
    //TODO: Write a tf listener node which listen to tf and publish current pose
    //Trajectory Generation Parameter 
    int look_ahead_{10};
    double vel_magnitude_;
    double total_time_;
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