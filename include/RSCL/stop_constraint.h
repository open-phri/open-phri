#pragma once

#include <memory>
#include <Eigen/Dense>

#include <definitions.h>
#include <constraint.h>

namespace RSCL {

namespace Constraints {

/* This must be the last constraint to be computed because 'force'
 * will be modified if 'external_force' norm exceed 'force_threshold' and left
 * untouched otherwise.
 */
class StopConstraint : public Constraint {
public:
	/***		Constructor & destructor		***/
	StopConstraint(
		Vector6dPtr external_force,
		doublePtr activation_force_threshold,
		doublePtr deactivation_force_threshold);
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

} // namespace Constraints

} // namespace RSCL
