#ifndef POWER_CONSTRAINT_HPP
#define POWER_CONSTRAINT_HPP

#include <memory>
#include <Eigen/Dense>

#include <definitions.h>
#include <constraint.h>

namespace ADamp {

class PowerConstraint : public Constraint {
public:
	/***		Constructor & destructor		***/
	PowerConstraint(
		std::shared_ptr<Matrix6d> tool_base_transform,
		std::shared_ptr<Matrix6d> damping_matrix,
		std::shared_ptr<Vector6d> reference_velocity,
		std::shared_ptr<Vector6d> force,
		std::shared_ptr<double> maximum_power);

	virtual ~PowerConstraint() = default;

	/***		Algorithm		***/
	virtual double compute() override;

private:
	std::shared_ptr<Matrix6d> tool_base_transform_;
	std::shared_ptr<Matrix6d> damping_matrix_;
	std::shared_ptr<Vector6d> reference_velocity_;
	std::shared_ptr<Vector6d> force_;
	std::shared_ptr<double> maximum_power_;
};

}

#endif /* POWER_CONSTRAINT_H */
