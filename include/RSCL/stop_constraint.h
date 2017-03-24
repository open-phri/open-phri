#pragma once

#include <memory>
#include <Eigen/Dense>

#include <definitions.h>
#include <constraint.h>

namespace RSCL {

class StopConstraint : public Constraint {
public:
	/***		Constructor & destructor		***/
	StopConstraint(
		Vector6dConstPtr external_force,
		doubleConstPtr activation_force_threshold,
		doubleConstPtr deactivation_force_threshold);

	virtual ~StopConstraint() = default;

	/***		Algorithm		***/
	virtual double compute() override;

private:
	Vector6dConstPtr external_force_;
	doubleConstPtr activation_force_threshold_;
	doubleConstPtr deactivation_force_threshold_;

	double previous_constraint_value_;
};

using StopConstraintPtr = std::shared_ptr<StopConstraint>;
using StopConstraintConstPtr = std::shared_ptr<const StopConstraint>;

} // namespace RSCL
