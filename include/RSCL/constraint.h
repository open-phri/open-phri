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
 * @file constraint.h
 * @author Benjamin Navarro
 * @brief Base classes definitions for constraint implementation
 * @date April 2014
 * @ingroup RSCL
 */

#pragma once

#include <memory>

namespace RSCL {

/** @enum RSCL::ConstraintType
 *  @brief Defines the type of a Constraint and how it will be treated by the SafetyController.
 */
enum class ConstraintType {
	Multiplicative, /**< Gets multiplied to the minimum function */
	Minimum         /**< Gets evaluated by the minimum function */
};

/** @brief Base class for all constraints.
 *  @details Provides a ConstraintType and a pure virtual compute method.
 */
class Constraint {
public:
	/**
	 * @brief Construct a constraint of a given type
	 * @param type The type of the constraint. See ConstraintType.
	 */
	Constraint(ConstraintType type);
	virtual ~Constraint() = default;

	/**
	 * @brief Provide the constraint's type
	 * @return The type. See ConstraintType.
	 */
	ConstraintType getType() const;

	/**
	 * @brief Compute the value associated with the constraint.
	 * @return The constraint's evaluated value.
	 */
	virtual double compute() = 0;

private:
	ConstraintType type_;
};

using ConstraintPtr = std::shared_ptr<Constraint>;
using ConstraintConstPtr = std::shared_ptr<const Constraint>;

} // namespace RSCL
