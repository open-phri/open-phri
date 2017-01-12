#include <stop_controller.h>

using namespace ADamp;

StopController::StopController(
	std::shared_ptr<Matrix6d> tool_base_transform,
	std::shared_ptr<Matrix6d> damping_matrix,
	std::shared_ptr<Vector6d> reference_velocity,
	std::shared_ptr<Vector6d> force,
	std::shared_ptr<Vector6d> external_force,
	std::shared_ptr<double> force_threshold,
	std::shared_ptr<double> force_threshold_hysteresis) :
	AdaptiveDamping(tool_base_transform,
	                damping_matrix,
	                reference_velocity,
	                force),
	force_(force),
	external_force_(external_force)
{
	stop_constraint_ = std::make_shared<StopConstraint>(
		damping_matrix,
		reference_velocity,
		force,
		external_force,
		force_threshold,
		force_threshold_hysteresis);
}

double StopController::computeConstraint() const {
	return stop_constraint_->compute();
}

Vector6d StopController::computeForces() const {
	return (*force_ = *external_force_);
}
