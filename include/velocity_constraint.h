#ifndef VELOCITY_CONSTRAINT_HPP
#define VELOCITY_CONSTRAINT_HPP

#include <memory>
#include <Eigen/Dense>

#include <definitions.h>
#include <constraint.h>

namespace ADamp {

class VelocityConstraint : public Constraint {
public:
	/***		Constructor & destructor		***/
	VelocityConstraint(
		std::shared_ptr<Matrix6d> tool_base_transform,
		std::shared_ptr<Matrix6d> damping_matrix,
		std::shared_ptr<Vector6d> reference_velocity,
		std::shared_ptr<Vector6d> force,
		std::shared_ptr<double> maximum_velocity);

	virtual ~VelocityConstraint() = default;

	/***		Algorithm		***/
	virtual double compute() override;

private:
	std::shared_ptr<Matrix6d> tool_base_transform_;
	std::shared_ptr<Matrix6d> damping_matrix_;
	std::shared_ptr<Vector6d> reference_velocity_;
	std::shared_ptr<Vector6d> force_;
	std::shared_ptr<double> maximum_velocity_;
};

}

#endif /* VELOCITY_CONSTRAINT_H */
