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
 * @file velocity_constraint.h
 * @author Benjamin Navarro
 * @brief Definition of the VelocityConstraint class
 * @date April 2017
 * @ingroup RSCL
 */

#pragma once

#include <RSCL/definitions.h>
#include <RSCL/constraints/constraint.h>

namespace RSCL {

/** @brief A constraint to limit the TCP velocity.
 */
class VelocityConstraint : public Constraint {
public:
	/***		Constructor & destructor		***/

	/**
	 * @brief Construct a velocity constraint with a given total and a maximum velocity.
	 * @param total_velocity A shared pointer to the total velocity. See SafetyController::getTotalVelocity.
	 * @param maximum_velocity A shared pointer to the maximum velocity allowed.
	 */
	VelocityConstraint(
		doubleConstPtr maximum_velocity);

	virtual ~VelocityConstraint() = default;

	/***		Algorithm		***/
	virtual double compute() override;

private:
	doubleConstPtr maximum_velocity_;
};

using VelocityConstraintPtr = std::shared_ptr<VelocityConstraint>;
using VelocityConstraintConstPtr = std::shared_ptr<const VelocityConstraint>;

} // namespace RSCL
