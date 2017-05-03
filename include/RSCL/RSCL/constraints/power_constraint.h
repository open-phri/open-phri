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
 * @file power_constraint.h
 * @author Benjamin Navarro
 * @brief Definition of the PowerConstraint class
 * @date April 2017
 * @ingroup RSCL
 */

#pragma once

#include <RSCL/definitions.h>
#include <RSCL/constraints/constraint.h>

namespace RSCL {

/** @brief A constraint to limit the exchanged power.
 *  @details The power considered is the dot product of the total velocity with the external force.
 *  The constraint is active only when the power is negative, which is when the robot pushes the environment.
 *  No limitation is applied when the robot is pushed by the environment.
 */
class PowerConstraint : public Constraint {
public:
	/***		Constructor & destructor		***/

	/**
	 * @brief Construct a power constraint with a given total velocity, external force and maximum power.
	 * @param total_velocity A shared pointer to the total velocity. See SafetyController::getTotalVelocity.
	 * @param external_force A shared pointer to the external force.
	 * @param maximum_power A shared pointer to the maximum power allowed.
	 */
	PowerConstraint(
		Vector6dConstPtr external_force,
		doubleConstPtr maximum_power);

	virtual ~PowerConstraint() = default;

	/***		Algorithm		***/
	virtual double compute() override;

	/**
	 * @brief Retrieve the computed power shared pointer.
	 * @return The shared pointer to the computed power.
	 */
	doubleConstPtr getPower() const;

private:
	Vector6dConstPtr external_force_;
	doubleConstPtr maximum_power_;

	doublePtr power_;
};

using PowerConstraintPtr = std::shared_ptr<PowerConstraint>;
using PowerConstraintConstPtr = std::shared_ptr<const PowerConstraint>;

} // namespace RSCL
