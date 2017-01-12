#include <power_constraint.h>

using namespace ADamp;
using namespace Eigen;

/***		Constructor & destructor		***/
PowerConstraint::PowerConstraint(
	std::shared_ptr<Matrix6d> tool_base_transform,
	std::shared_ptr<Matrix6d> damping_matrix,
	std::shared_ptr<Vector6d> reference_velocity,
	std::shared_ptr<Vector6d> force,
	std::shared_ptr<double> maximum_power) :
	tool_base_transform_(tool_base_transform),
	damping_matrix_(damping_matrix),
	reference_velocity_(reference_velocity),
	force_(force),
	maximum_power_(maximum_power)
{

}

/***		Algorithm		***/
double PowerConstraint::compute() {
	double alpha = 1.;
	Vector3d feq = force_->block<3,1>(0,0);

	if(feq.norm() > 1e-3) {
		Matrix3d R_b_t = tool_base_transform_->block<3,3>(0,0);
		Matrix3d Bt_inv = damping_matrix_->block<3,3>(0,0).inverse();
		Vector3d vr = R_b_t * reference_velocity_->block<3,1>(0,0);
		double pm = (*maximum_power_);

		alpha = (pm - feq.transpose()*R_b_t*vr) / (feq.transpose()*Bt_inv*feq);
	}

	return alpha;
}
