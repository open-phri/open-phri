#ifndef STOP_CONSTRAINT_HPP
#define STOP_CONSTRAINT_HPP

#include <memory>
#include <Eigen/Dense>

#include <definitions.h>
#include <constraint.h>

namespace ADamp {

/* This must be the last constraint to be computed because 'force'
 * will be modified if 'external_force' norm exceed 'force_threshold' and left
 * untouched otherwise.
 */
class StopConstraint : public Constraint {
public:
	/***		Constructor & destructor		***/
	StopConstraint(
		std::shared_ptr<Matrix6d> damping_matrix,
		std::shared_ptr<Vector6d> reference_velocity,
		std::shared_ptr<Vector6d> force,
		std::shared_ptr<Vector6d> external_force,
		std::shared_ptr<double> force_threshold,
		std::shared_ptr<double> force_threshold_hysteresis);

	virtual ~StopConstraint() = default;

	/***		Algorithm		***/
	virtual double compute() override;

private:
	std::shared_ptr<Matrix6d> damping_matrix_;
	std::shared_ptr<Vector6d> reference_velocity_;
	std::shared_ptr<Vector6d> force_;
	std::shared_ptr<Vector6d> external_force_;
	std::shared_ptr<double> force_threshold_;
	std::shared_ptr<double> force_threshold_hysteresis_;

	bool robot_stopped_;
};

}

#endif /* STOP_CONSTRAINT_H */
