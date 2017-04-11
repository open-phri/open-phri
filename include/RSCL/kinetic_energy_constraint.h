#pragma once

#include <RSCL/velocity_constraint.h>

namespace RSCL {

class KineticEnergyConstraint : public VelocityConstraint {
public:
	/***		Constructor & destructor		***/
	KineticEnergyConstraint(
		Vector6dConstPtr total_velocity,
		doubleConstPtr mass,
		doubleConstPtr maximum_kinetic_energy);

	virtual ~KineticEnergyConstraint() = default;

	/***		Algorithm		***/
	virtual double compute() override;

private:
	Vector6dConstPtr total_velocity_;
	doubleConstPtr mass_;
	doubleConstPtr maximum_kinetic_energy_;

	doublePtr maximum_velocity_;
};

using KineticEnergyConstraintPtr = std::shared_ptr<KineticEnergyConstraint>;
using KineticEnergyConstraintConstPtr = std::shared_ptr<const KineticEnergyConstraint>;

} // namespace RSCL
