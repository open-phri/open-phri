#include <OpenPHRI/constraints/acceleration_constraint.h>

using namespace phri;

using namespace Eigen;

/***		Constructor & destructor		***/
AccelerationConstraint::AccelerationConstraint(
	doubleConstPtr maximum_acceleration,
	double sample_time) :
	maximum_acceleration_(maximum_acceleration),
	sample_time_(sample_time)
{
}

/***		Algorithm		***/
double AccelerationConstraint::compute() {
	double constraint = 1.;
	double v_norm = robot_->controlPointTotalVelocity()->block<3,1>(0,0).norm();

	if(v_norm > 0.) {
		double prev_v_norm = robot_->controlPointVelocity()->block<3,1>(0,0).norm();
		double vmax = prev_v_norm + std::abs(*maximum_acceleration_) * sample_time_;
		constraint = vmax / v_norm;
	}

	return constraint;
}
