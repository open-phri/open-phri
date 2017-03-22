#pragma once

#include <definitions.h>
#include <constraint.h>

namespace RSCL {

namespace Constraints {

class PowerConstraint : public Constraint {
public:
	/***		Constructor & destructor		***/
	PowerConstraint(
		Vector6dConstPtr total_velocity,
		Vector6dConstPtr external_force,
		doubleConstPtr maximum_power);

	virtual ~PowerConstraint() = default;

	/***		Algorithm		***/
	virtual double compute() override;

private:
	Vector6dConstPtr total_velocity_;
	Vector6dConstPtr external_force_;
	doubleConstPtr maximum_power_;
};

using PowerConstraintPtr = std::shared_ptr<PowerConstraint>;
using PowerConstraintConstPtr = std::shared_ptr<const PowerConstraint>;

} // namespace Constraints

} // namespace RSCL
