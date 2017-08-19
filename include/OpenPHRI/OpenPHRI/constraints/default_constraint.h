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
 * @file default_constraint.h
 * @author Benjamin Navarro
 * @brief Definition of the DefaultConstraint class
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/constraints/constraint.h>

namespace OpenPHRI {

/** @brief Default constraint. Always evaluate to one.
 */
class DefaultConstraint : public Constraint {
public:
	/***		Constructor & destructor		***/

	/**
	 * @brief Construct a default constraint of a given type
	 */
	DefaultConstraint() = default;

	virtual ~DefaultConstraint() = default;

	/***		Algorithm		***/
	virtual double compute() override;
};

using DefaultConstraintPtr = std::shared_ptr<DefaultConstraint>;
using DefaultConstraintConstPtr = std::shared_ptr<const DefaultConstraint>;

} // namespace OpenPHRI
