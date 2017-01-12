#ifndef STOP_CONTROLLER_H
#define STOP_CONTROLLER_H

#include <adaptive_damping.h>
#include <stop_constraint.h>

namespace ADamp {

class StopController : public AdaptiveDamping {
public:
	StopController(
		std::shared_ptr<Matrix6d> tool_base_transform,
		std::shared_ptr<Matrix6d> damping_matrix,
		std::shared_ptr<Vector6d> reference_velocity,
		std::shared_ptr<Vector6d> force,
		std::shared_ptr<Vector6d> external_force,
		std::shared_ptr<double> force_threshold,
		std::shared_ptr<double> force_threshold_hysteresis);
	virtual ~StopController() = default;

protected:
	virtual double computeConstraint() const override;
	virtual Vector6d computeForces() const override;

private:
	std::shared_ptr<Vector6d> force_;
	std::shared_ptr<Vector6d> external_force_;
	std::shared_ptr<StopConstraint> stop_constraint_;
};

}

#endif /* STOP_CONTROLLER_H */
