#include "ncpr/trajectory_tracker.h"

 
Eigen::Vector2d TrajectoryTracker::computeTrackingControl
	(TrajectoryType::OutputType pos, TrajectoryType::OutputType vel, TrajectoryType::Ptr trajectory_ptr, double time)
{
	// car length
	double L = 1.0;
	// desired pose
	OutputType pose_d = trajectory_ptr->getDesired(time,0);
	//desired velocity
	OutputType vel_d = trajectory_ptr->getDesired(time,1);
	// desired acceleration
	OutputType vel_dd = trajectory_ptr->getDesired(time,2);

	OutputType z_pos = pos - pose_d;
	OutputType z_vel = vel - vel_d;

	//set the virtual input
	OutputType v = vel_dd - kp*z_pos - kd*z_vel;

	double theta = atan2(vel_d(1,0), vel_d(0,0));
	Eigen::Matrix2d Rot;
	Rot << cos(theta), -sin(theta),
		   sin(theta),  cos(theta);

	// control law
	// u_temp = [du1 , u1^1 * tan(u2)/L]
	OutputType u_temp = Rot.transposeInPlace()*v;

	// u1 = velocity, u2 = steering angle
	Eigen::Vector2d u;
	u(0) = u_temp(0,0)*time;
	u(1) = atan2(u_temp(1,0)*L, pow(u(0),2));

	return u;
}