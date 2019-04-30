#ifndef TRAJECTORY_TRACKER_H_
#define TRAJECTORY_TRACKER_H_

#include "ncpr/common.h"
#include "ncpr/poly_trajectory.h"

namespace ncpr{
  class TrajectoryTracker{
  public:
    explicit TrajectoryTracker(TrajectoryType::Ptr trajectory_ptr):trajectory_ptr_(trajectory_ptr){
      
    }
    using TrajectoryType = PolyTrajectory<double,2,3>;
    /*!
     * @brief Given current output position and velocity, compute control to track the trajectory 
     * @param pos Flat output position 
     * @param vel Flat output velocity 
     * @return control output to track the trajectory 
     */
    Eigen::Vector2d computeTrackingControl(TrajectoryType::OutputType pos, TrajectoryType::OutputType vel, TrajectoryType::Ptr trajectory_ptr);
  private:
    //control parameter 
    double kp{};
    double kd{};
  private:
    TrajectoryType::Ptr trajectory_ptr_;
  };
}


#endif 
