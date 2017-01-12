#include <stop_constraint.h>
#include <iostream>

using namespace ADamp;
using namespace Eigen;

/***		Constructor & destructor		***/
StopConstraint::StopConstraint(
	std::shared_ptr<Matrix6d> damping_matrix,
	std::shared_ptr<Vector6d> reference_velocity,
	std::shared_ptr<Vector6d> force,
	std::shared_ptr<Vector6d> external_force,
	std::shared_ptr<double> force_threshold,
	std::shared_ptr<double> force_threshold_hysteresis) :
	damping_matrix_(damping_matrix),
	reference_velocity_(reference_velocity),
	force_(force),
	external_force_(external_force),
	force_threshold_(force_threshold),
	force_threshold_hysteresis_(force_threshold_hysteresis),
	robot_stopped_(false)
{

}

/***		Algorithm		***/
double StopConstraint::compute() {
	double alpha = 1.;
	double ft = *force_threshold_;
	double ft_hyst = *force_threshold_hysteresis_;
	Vector6d hext = *external_force_;
	bool constraint_activated;

	constraint_activated = (hext.norm() >= ft)or(robot_stopped_ and (hext.norm() >= (ft - ft_hyst)));

	if(constraint_activated) {
		Vector6d dxr = *reference_velocity_;
		Vector6d heq = dxr/dxr.norm();
		Matrix6d B_inv = damping_matrix_->inverse();

		*force_ = heq; // set the equivalent force to the right value in order to be able to stop the robot
		alpha = -dxr.norm()/(B_inv*heq).norm();

		robot_stopped_ = true;
	}
	else {
		robot_stopped_ = false;
	}


	return alpha;
}
