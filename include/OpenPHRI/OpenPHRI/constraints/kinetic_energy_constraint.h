/*
 *  Copyright (C) 2017 Benjamin Navarro <contact@bnavarro.info>
 *
 *  This file is part of OpenPHRI <https://gite.lirmm.fr/navarro/OpenPHRI>.
 *
 *  OpenPHRI is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  OpenPHRI is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with OpenPHRI.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file kinetic_energy_constraint.h
 * @author Benjamin Navarro
 * @brief Definition of the KineticEnergyConstraint class
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/constraints/velocity_constraint.h>

namespace phri {

/** @brief A constraint to limit the robot's kinetic energy.
 *  @details Works with a point mass model. For a manipulator, see ManipulatorEquivalentMass to get its equivalent mass.
 */
class KineticEnergyConstraint : public VelocityConstraint {
public:
	/***		Constructor & destructor		***/

	/**
	 * @brief Construct the kinematic energy constraint.
	 * @param mass The mass or equivalent mass of the robot.
	 * @param maximum_kinetic_energy The maximum kinetic energy allowed.
	 */
	KineticEnergyConstraint(
		doubleConstPtr mass,
		doubleConstPtr maximum_kinetic_energy);

	virtual ~KineticEnergyConstraint() = default;

	/***		Algorithm		***/
	virtual double compute() override;

private:
	doubleConstPtr mass_;
	doubleConstPtr maximum_kinetic_energy_;

	doublePtr kinetic_energy_maximum_velocity_;
};

using KineticEnergyConstraintPtr = std::shared_ptr<KineticEnergyConstraint>;
using KineticEnergyConstraintConstPtr = std::shared_ptr<const KineticEnergyConstraint>;

} // namespace phri
