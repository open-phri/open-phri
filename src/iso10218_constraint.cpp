#include <iso10218_constraint.h>

using namespace ADamp;
using namespace Eigen;

/***		Constructor & destructor		***/
ISO10218Constraint::ISO10218Constraint(
	std::shared_ptr<Matrix6d> tool_base_transform,
	std::shared_ptr<Matrix6d> damping_matrix,
	std::shared_ptr<Vector6d> reference_velocity,
	std::shared_ptr<Vector6d> force,
	std::shared_ptr<double> maximum_velocity,
	std::shared_ptr<double> maximum_power,
	std::shared_ptr<double> maximum_force) :
	tool_base_transform_(tool_base_transform),
	damping_matrix_(damping_matrix),
	reference_velocity_(reference_velocity),
	force_(force),
	maximum_velocity_(maximum_velocity),
	maximum_power_(maximum_power),
	maximum_force_(maximum_force)
{
	velocity_constaint_ = std::make_shared<VelocityConstraint>(
		tool_base_transform,
		damping_matrix,
		reference_velocity,
		force,
		maximum_velocity);

	power_constaint_ = std::make_shared<PowerConstraint>(
		tool_base_transform,
		damping_matrix,
		reference_velocity,
		force,
		maximum_power);
}

/***		Algorithm		***/
double ISO10218Constraint::compute() {
	double alpha = 1.;
	double fm = (*maximum_force_);
	Vector3d feq = force_->block<3,1>(0,0);
	double alpha_v = velocity_constaint_->compute();
	double alpha_p = power_constaint_->compute();

	if(feq.norm() <= fm) {
		alpha = std::min(std::min(alpha_v, alpha_p), 1.);
	}
	else {
		alpha = std::min(alpha_v, alpha_p);
	}

	return alpha;
}
