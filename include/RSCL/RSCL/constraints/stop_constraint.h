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
 * @file stop_constraint.h
 * @author Benjamin Navarro
 * @brief Definition of the StopConstraint class
 * @date April 2017
 * @ingroup RSCL
 */

#pragma once

#include <RSCL/definitions.h>
#include <RSCL/constraints/constraint.h>

namespace RSCL {

/** @brief A constraint to stop the robot when a contant occurs.
 *  @details Two thresholds are used. The robot is stopped when the external force is above the activation threshold and
 *  released when the external force gets below the deactivation threshold.
 */
class StopConstraint : public Constraint {
public:
	/***		Constructor & destructor		***/

	/**
	 * @brief Construct a stop constraint with a given external force, activation and deactivation threshold.
	 * @param external_force A shared pointer to the external force.
	 * @param activation_force_threshold A shared pointer to the activation threshold.
	 * @param deactivation_force_threshold A shared pointer to the deactivation threshold.
	 */
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
