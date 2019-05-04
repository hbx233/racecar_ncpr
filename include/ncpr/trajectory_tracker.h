#ifndef TRAJECTORY_TRACKER_H_
#define TRAJECTORY_TRACKER_H_

#include "ncpr/common.h"
#include "ncpr/poly_trajectory.h"

namespace ncpr{
  class TrajectoryTracker{
  public:
    using Ptr = shared_ptr<TrajectoryTracker>;
    using TrajectoryType = PolyTrajectory<double,2,3>;
    explicit TrajectoryTracker(TrajectoryType::Ptr trajectory_ptr):trajectory_ptr_(trajectory_ptr){
      
    }

    /*!
     * @brief Given current output position and velocity, compute control to track the trajectory 
     * @param pos Flat output position 
     * @param vel Flat output velocity 
     * @return control output to track the trajectory 
     */
    
    Eigen::Vector3d computeTrackingControl(TrajectoryType::OutputType pos, TrajectoryType::OutputType vel, double time);
    /*!
     * @brief compute and return the elapsed time of tracking of local trajectory started from start_time_
     */
    double getTrackingElapsedTime();
  private:
    //control parameter 
    double velocity_;
    double car_length_;
    double kp{};
    double kd{};
  private:
    double start_time_;
    double prev_time_;
    double curr_time_;
    unsigned long trajectory_id_{0}; //ID 0 is initial invalid trajectory 
    TrajectoryType::Ptr trajectory_ptr_;
  };
}


#endif 
