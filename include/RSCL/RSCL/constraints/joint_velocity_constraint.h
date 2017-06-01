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
 * @file joint_velocity_constraint.h
 * @author Benjamin Navarro
 * @brief Definition of the JointVelocityConstraint class
 * @date June 2017
 * @ingroup RSCL
 */

#pragma once

#include <RSCL/definitions.h>
#include <RSCL/constraints/constraint.h>

namespace RSCL {

/** @brief A constraint to limit the joint velocities.
 */
class JointVelocityConstraint : public Constraint {
public:
	/***		Constructor & destructor		***/

	/**
	 * @brief Construct a joint velocity constraint with a given a vector of maximum velocities.
	 * @param maximum_velocity A shared pointer to the maximum velocities allowed.
	 */
	JointVelocityConstraint(
		VectorXdConstPtr maximum_velocities);

	virtual ~JointVelocityConstraint() = default;

	/***		Algorithm		***/
	virtual double compute() override;

private:
	VectorXdConstPtr maximum_velocities_;
};

using JointVelocityConstraintPtr = std::shared_ptr<JointVelocityConstraint>;
using JointVelocityConstraintConstPtr = std::shared_ptr<const JointVelocityConstraint>;

} // namespace RSCL
