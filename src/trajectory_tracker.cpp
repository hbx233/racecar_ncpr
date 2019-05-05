#include "ncpr/trajectory_tracker.h"

namespace ncpr{
  Eigen::Vector3d TrajectoryTracker::computeTrackingControl
  (TrajectoryType::OutputType pos, TrajectoryType::OutputType vel, double time)
  {  
    if(trajectory_ptr_->getID() != trajectory_id_){
      //The local trajectory is updated, need to switch and clear all caches 
      //update trajectory_id_
      trajectory_id_  = trajectory_ptr_->getID();
      //update current trajectory start time
      start_time_ = time;
      curr_time_ = time;
      prev_time_ = time;
    }
    prev_time_ = curr_time_;
    //current local trajectory tracking time is the absolute time - local trajectory start time;
    curr_time_ = time - start_time_;
    double dt = curr_time_ - prev_time_ <0 ? 0: curr_time_ - prev_time_;
    // car length
    double L = car_length_;
    // desired pose
    typename TrajectoryType::OutputType pose_d = trajectory_ptr_->getDesired(curr_time_,0);
    //desired velocity
    typename TrajectoryType::OutputType vel_d = trajectory_ptr_->getDesired(curr_time_,1);
    // desired acceleration
    typename TrajectoryType::OutputType vel_dd = trajectory_ptr_->getDesired(curr_time_,2);
    
    typename TrajectoryType::OutputType z_pos = pos - pose_d;
    typename TrajectoryType::OutputType z_vel = vel - vel_d;
    
    //set the virtual input
    typename TrajectoryType::OutputType v = vel_dd - kp*z_pos - kd*z_vel;
    
    cout<<"[Debug] V(0): "<<v(0)<<endl;
    cout<<"[Debug] V(1): "<<v(1)<<endl;

    double theta = atan2(vel(1,0), vel(0,0));
    Eigen::Matrix2d Rot;
    Rot << cos(theta), -sin(theta),
    sin(theta),  cos(theta);
    
    // control law
    // u_temp = [du1 , u1^2 * tan(u2)/L]
    Rot.transposeInPlace();
    TrajectoryType::OutputType u_temp = Rot * v;
    
    // u1 = velocity, u2 = steering angle
    Eigen::Vector3d u;
    double old_velocity = std::sqrt(std::pow(vel(0),2) + std::pow(vel(1),2));
    cout<<"[Debug] Old Velocity: "<<old_velocity<<endl;
    cout<<"[Debug] dt: "<<dt<<endl;

    velocity_ = old_velocity + u_temp(0,0)*dt;
    u(0) = u_temp(0,0); //acceraltaion;
    u(1) = velocity_; //velocity
    u(2) = atan2(u_temp(1,0)*L, pow(old_velocity,2)); //steering anlge.
    cout<<"Acceleration: "<<u(0)<<endl;
    cout<<"Velocity: "<<u(1)<<endl;
    cout<<"Steering Angle: "<<u(2)<<endl;
    return u;
  }
double TrajectoryTracker::getTrackingElapsedTime()
{
  return curr_time_-start_time_;
}
  
}
