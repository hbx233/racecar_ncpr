#ifndef POLY_TRAJ_H_
#define POLY_TRAJ_H_
#include <ncpr/common.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>

namespace ncpr{
  template <typename T, int OutputDim, int BasisOrder>
  class PolyTrajectory{
  public:
    //private constructor, shouldn't directly construct PolyTrajectory 
    PolyTrajectory(): coeff_(CoeffType::Zero()), total_time_(0) {};
  public:
    //Define Useful types
    using BasisType = Eigen::Matrix<T, BasisOrder+1, 1>;
    using OutputType = Eigen::Matrix<T, OutputDim, 1>;
    using CoeffType = Eigen::Matrix<T, OutputDim, BasisOrder+1>;
    using Ptr = shared_ptr<PolyTrajectory<T,OutputDim,BasisOrder>>;
    /*!
     * @brief Get desired value of provided order of trajectory at provided time 
     * @param time current time 
     * @param order the derivative order of trajectory 
     * @return the desired value for this trajectory 
     */
    //Eigen::Matrix<T,OutputDim,1> getDesired(double time, int order);
    OutputType getDesired(const double& time, const int& order);
    /*!
     * @brief convert the poly trajectory to Path message type, used to visualize the trajectory in RViz
     * @param time_interval the time interval that used to generate the Path from current trajectory
     * @return sampled trajectory in Path type 
     */
    nav_msgs::Path generatePath(const double& time_interval);
    /*!
     * @brief fit polynomial trajectory given start and goal output and start and goal velocity output
     *        Assume higher order of derivative of start and goal is zero, eg. acceleration & jerk are zero 
     * @param start start state
     * @param goal goal state
     * @param start_vel start velocity
     * @param goal goal velocity 
     * @param total_time total_time needed for the trajectory fitting 
     */
    void fitPolyTrajectory(const OutputType& start, const OutputType& goal, const OutputType& start_vel, const OutputType& goal_vel, const double& total_time);
    /*!
     * @brief Geter for coefficient
     */
    CoeffType getCoefficient();
  private:
    /*!
     * @brief Helper function that returns basis vector of poly trajectory 
     * @param time current time
     * @param order the derivative order of trajectory with respect to time 
     * @return the basis vector 
     */
    BasisType getBasisVector(const double& time, const int& order);
    bool valid_trajectory_{false};
    OutputType  start_;
    OutputType goal_;
    double total_time_;
    CoeffType coeff_;
  };
template <typename T, int OutputDim, int BasisOrder> 
typename PolyTrajectory<T,OutputDim, BasisOrder>::OutputType PolyTrajectory<T, OutputDim, BasisOrder>::getDesired(const double& time, const int& order)
{
  //construct basis vector in provided derivative order of time 
  Eigen::Matrix<T, BasisOrder+1,1> basis = getBasisVector(time, order);
  //multiply basis vectory with coefficient to get desired output 
  return coeff_ * basis;
}

template <typename T, int OutputDim, int BasisOrder>
nav_msgs::Path PolyTrajectory<T, OutputDim, BasisOrder>::generatePath(const double& time_interval){
  double t = 0;

  nav_msgs::Path path;
  geometry_msgs::Pose pose;
  geometry_msgs::PoseStamped pose_stamped;
  //TODO: Need Header
  std_msgs::Header header_msg;
  header_msg.stamp = ros::Time::now();
  header_msg.frame_id = "base_link";
  path.header = header_msg;

  while(t<total_time_){
    OutputType pose_state = getDesired(t,0);
    cout << pose_state(0) << endl;

    pose.position.x = pose_state(0);
    pose.position.y = pose_state(1);
    
    pose_stamped.pose = pose;
    path.poses.push_back(pose_stamped);
    t+=time_interval;
  }
  return path;
}
template <typename T, int OutputDim, int BasisOrder>
void PolyTrajectory<T, OutputDim, BasisOrder>::fitPolyTrajectory
    (const OutputType& start, const OutputType& goal, const OutputType& start_vel, const OutputType& goal_vel, const double& total_time){
  //Construct Output matrix from start and goal state & velocity 
  CoeffType Y = CoeffType::Zero();
  //NOTE: Only use the desired output and desired first order derivative of output 
  //to fit trajectory, higher order of derivative are assumed to be zero 
  Y.col(0) = start;
  Y.col(1) = start_vel;
  Y.col((BasisOrder+1)/2) = goal;
  Y.col((BasisOrder+1)/2+1) = goal_vel;
  cout<<Y<<endl;
  //construct basis matrix 
  Eigen::Matrix<T, BasisOrder+1, BasisOrder+1> Lambda = Eigen::Matrix<T,BasisOrder+1,BasisOrder+1>::Zero();
  for(int c=0; c<(BasisOrder+1)/2; c++){
    Lambda.col(c) = getBasisVector(0,c);
    Lambda.col(c+(BasisOrder+1)/2) = getBasisVector(total_time,c);
  }
  cout<<Lambda<<endl;
  //compute coefficient marix 
  if(Lambda.determinant()<0.001){
    std::cerr<<"Singularity"<<endl;
  } else{
    coeff_ = Y * Lambda.inverse();
    total_time_ = total_time;
    valid_trajectory_=true;
  }
}

template <typename T, int OutputDim, int BasisOrder> 
typename PolyTrajectory<T,OutputDim,BasisOrder>::BasisType PolyTrajectory<T, OutputDim,BasisOrder>::getBasisVector(const double& time, const int& order){
  BasisType basis = BasisType::Zero();
  for(int i=0; i<=BasisOrder-order; i++){
    T c=1;
    for(int j=0; j<order; j++){
      c*=BasisOrder-i-j;
    }
    basis(i) = c*std::pow(time,BasisOrder-order-i);
  }
  return basis;
}

template <typename T, int OutputDim, int BasisOrder> 
typename PolyTrajectory<T, OutputDim, BasisOrder>::CoeffType PolyTrajectory<T, OutputDim, BasisOrder>::getCoefficient(){
  return coeff_;
}
}



#endif