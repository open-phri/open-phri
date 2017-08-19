#include <OpenPHRI/constraints/kinetic_energy_constraint.h>

using namespace OpenPHRI;
using namespace std;

/***		Constructor & destructor		***/
KineticEnergyConstraint::KineticEnergyConstraint(
	doubleConstPtr mass,
	doubleConstPtr maximum_kinetic_energy) :
	mass_(mass),
	maximum_kinetic_energy_(maximum_kinetic_energy)
{
	kinetic_energy_maximum_velocity_ = make_shared<double>(0.);
	maximum_velocity_ = kinetic_energy_maximum_velocity_;
}

/***		Algorithm		***/
double KineticEnergyConstraint::compute() {
	*kinetic_energy_maximum_velocity_ = std::sqrt(2. * *maximum_kinetic_energy_ / *mass_);

	return VelocityConstraint::compute();
}
