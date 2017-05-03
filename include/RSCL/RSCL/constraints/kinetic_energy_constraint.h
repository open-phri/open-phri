/*
 *  Copyright (C) 2017 Benjamin Navarro <contact@bnavarro.info>
 *
 *  This file is part of RSCL <https://gite.lirmm.fr/navarro/RSCL>.
 *
 *  RSCL is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  RSCL is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with RSCL.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file kinetic_energy_constraint.h
 * @author Benjamin Navarro
 * @brief Definition of the KineticEnergyConstraint class
 * @date April 2017
 * @ingroup RSCL
 */

#pragma once

#include <RSCL/constraints/velocity_constraint.h>

namespace RSCL {

/** @brief A constraint to limit the robot's kinetic energy.
 *  @details Works with a point mass model. For a manipulator, see ManipulatorEquivalentMass to get its equivalent mass.
 */
class KineticEnergyConstraint : public VelocityConstraint {
public:
	/***		Constructor & destructor		***/

	/**
	 * @brief Construct the kinematic energy constraint.
	 * @param total_velocity The total velocity computed by the controller. See SafetyController::getTotalVelocity.
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

	doublePtr maximum_velocity_;
};

using KineticEnergyConstraintPtr = std::shared_ptr<KineticEnergyConstraint>;
using KineticEnergyConstraintConstPtr = std::shared_ptr<const KineticEnergyConstraint>;

} // namespace RSCL
