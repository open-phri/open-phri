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
 * @file constraint.h
 * @author Benjamin Navarro
 * @brief Base classes definitions for constraint implementation
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/robot.h>
#include <OpenPHRI/fwd_decl.h>

namespace OpenPHRI {

/** @brief Base class for all constraints.
 *  @details Provides a ConstraintType and a pure virtual compute method.
 */
class Constraint {
public:
	/**
	 * @brief Construct a constraint of a given type
	 * @param type The type of the constraint. See ConstraintType.
	 */
	Constraint() = default;
	virtual ~Constraint() = default;

	/**
	 * @brief Compute the value associated with the constraint.
	 * @return The constraint's evaluated value.
	 */
	virtual double compute() = 0;

	/**
	 * @brief Call operator, shortcut for compute.
	 * @return The constraint's evaluated value.
	 */
	virtual double operator()() final;

	/**
	 * @brief Set the robot to work with. Should not be set by the user.
	 * @param robot The robot.
	 */
	virtual void setRobot(RobotConstPtr robot);

protected:
	RobotConstPtr robot_;
};

using ConstraintPtr = std::shared_ptr<Constraint>;
using ConstraintConstPtr = std::shared_ptr<const Constraint>;

} // namespace OpenPHRI
