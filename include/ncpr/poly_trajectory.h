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
PolyTrajectory::OutputType PolyTrajectory<T, OutputDim, BasisOrder>::getDesired(const double& time, const int& order)
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
  //TODO: Need Header??
  while(t<total_time_){
    path.poses.push_back(getDesired(t,0));
    t+=time_interval;
  }
  return path;
}
template <typename T, int OutputDim, int BasisOrder>
void PolyTrajectory<T, OutputDim, BasisOrder>::fitPolyTrajectory
    (const OutputType& start, const OutputType& goal, const OutputType& start_vel, const OutputType& goal_vel, const double& total_time){
  //Construct Output matrix from start and goal state & velocity 
  PolyTrajectory::CoeffType Y = PolyTrajectory::CoeffType::Zero();
  Y.col(0) = start;
  Y.col(1) = start_vel;
  Y.col(BasisOrder/2) = goal;
  Y.col(BasisOrder/2) = goal_vel;
  //construct basis matrix 
  Eigen::Matrix<T, BasisOrder+1, BasisOrder+1> Lambda = Eigen::Matrix<T,BasisOrder+1,BasisOrder+1>::Zero();
  for(int c=0; c<BasisOrder/2; c++){
    Lambda.col(c) = getBasisVector(0,c);
    Lambda.col(c+BasisOrder/2) = getBasisVector(total_time,c);
  }
  //compute coefficient marix 
  if(Lambda.determinant()<0.001){
    std::cerr<<"Singularity"<<endl;
  } else{
    coeff_ = Y * Lambda.inverse();
    valid_trajectory_=true;
  }
}

template <typename T, int OutputDim, int BasisOrder> 
PolyTrajectory::BasisType PolyTrajectory<T, OutputDim,BasisOrder>::getBasisVector(const double& time, const int& order){
  PolyTrajectory::BasisType basis = PolyTrajectory::BasisType::Zero();
  for(int i=0; i<=BasisOrder-order; i++){
    T c=1;
    for(int j=0; j<order; j++){
      c*=BasisOrder-i-j;
    }
    basis(i) = c*std::pow(t,BasisOrder-order-i);
  }
  return basis;
}

}



#endif