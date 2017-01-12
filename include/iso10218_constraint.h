#ifndef ISO10218_CONSTRAINT_HPP
#define ISO10218_CONSTRAINT_HPP

#include <memory>
#include <Eigen/Dense>

#include <definitions.h>
#include <velocity_constraint.h>
#include <power_constraint.h>

namespace ADamp {

class ISO10218Constraint : public Constraint {
public:
	/***		Constructor & destructor		***/
	ISO10218Constraint(
		std::shared_ptr<Matrix6d> tool_base_transform,
		std::shared_ptr<Matrix6d> damping_matrix,
		std::shared_ptr<Vector6d> reference_velocity,
		std::shared_ptr<Vector6d> force,
		std::shared_ptr<double> maximum_velocity,
		std::shared_ptr<double> maximum_power,
		std::shared_ptr<double> maximum_force);

	virtual ~ISO10218Constraint() = default;

	/***		Algorithm		***/
	virtual double compute() override;

private:
	std::shared_ptr<Matrix6d> tool_base_transform_;
	std::shared_ptr<Matrix6d> damping_matrix_;
	std::shared_ptr<Vector6d> reference_velocity_;
	std::shared_ptr<Vector6d> force_;
	std::shared_ptr<double> maximum_velocity_;
	std::shared_ptr<double> maximum_power_;
	std::shared_ptr<double> maximum_force_;

	std::shared_ptr<VelocityConstraint> velocity_constaint_;
	std::shared_ptr<PowerConstraint> power_constaint_;
};

}

#endif /* ISO10218_CONSTRAINT_H */
