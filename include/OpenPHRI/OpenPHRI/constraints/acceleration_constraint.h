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
 * @file acceleration_constraint.h
 * @author Benjamin Navarro
 * @brief Definition of the AccelerationConstraint class
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/constraints/constraint.h>

namespace OpenPHRI {

/** @brief A constraint to limit the TCP acceleration.
 */
class AccelerationConstraint : public Constraint {
public:
	/***		Constructor & destructor		***/

	/**
	 * @brief Construct a acceleration constraint with a given total and a maximum acceleration.
	 * @param maximum_acceleration A shared pointer to the maximum acceleration allowed.
	 */
	AccelerationConstraint(
		doubleConstPtr maximum_acceleration,
		double sample_time);

	virtual ~AccelerationConstraint() = default;

	/***		Algorithm		***/
	virtual double compute() override;

private:
	doubleConstPtr maximum_acceleration_;
	double sample_time_;
};

using AccelerationConstraintPtr = std::shared_ptr<AccelerationConstraint>;
using AccelerationConstraintConstPtr = std::shared_ptr<const AccelerationConstraint>;

} // namespace OpenPHRI
