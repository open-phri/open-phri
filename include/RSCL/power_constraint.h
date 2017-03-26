#pragma once

#include <RSCL/definitions.h>
#include <RSCL/constraint.h>

namespace RSCL {

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

	doubleConstPtr getPower() const;

private:
	Vector6dConstPtr total_velocity_;
	Vector6dConstPtr external_force_;
	doubleConstPtr maximum_power_;

	doublePtr power_;
};

using PowerConstraintPtr = std::shared_ptr<PowerConstraint>;
using PowerConstraintConstPtr = std::shared_ptr<const PowerConstraint>;

} // namespace RSCL
