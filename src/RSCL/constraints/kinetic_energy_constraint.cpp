#include <RSCL/constraints/kinetic_energy_constraint.h>

using namespace RSCL;
using namespace std;

/***		Constructor & destructor		***/
KineticEnergyConstraint::KineticEnergyConstraint(
	Vector6dConstPtr total_velocity,
	doubleConstPtr mass,
	doubleConstPtr maximum_kinetic_energy) :
	VelocityConstraint(total_velocity, maximum_velocity_),
	total_velocity_(total_velocity),
	mass_(mass),
	maximum_kinetic_energy_(maximum_kinetic_energy)
{
	maximum_velocity_ = make_shared<double>(0.);
}

/***		Algorithm		***/
double KineticEnergyConstraint::compute() {
	*maximum_velocity_ = std::sqrt(2. * *maximum_kinetic_energy_ / *mass_);

	return VelocityConstraint::compute();
}
