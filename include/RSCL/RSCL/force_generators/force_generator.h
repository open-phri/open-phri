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
 * @file force_generator.h
 * @author Benjamin Navarro
 * @brief Base class definition for force generators
 * @date April 2017
 * @ingroup RSCL
 */

#pragma once

#include <RSCL/definitions.h>
#include <RSCL/robot.h>
#include <RSCL/fwd_decl.h>

namespace RSCL {

/** @brief Base class for all force generators.
 *  @details Provides a pure virtual compute method.
 */
class ForceGenerator {
public:
	~ForceGenerator() = default;

	/**
	 * @brief Compute the value associated with the force generator.
	 * @return The force generator's evaluated value.
	 */
	virtual Vector6d compute();

	/**
	 * @brief Call operator, shortcut for compute().
	 * @return The force generator's evaluated value.
	 */
	virtual Vector6d operator()() final;

protected:
	/**
	 * @brief Construct a force generator
	 * @param frame The reference frame in which the force is expressed.
	 */
	ForceGenerator(ReferenceFrame frame);

	/**
	 * @brief Transform the given force in the TCP frame, if necessary.
	 * @param force The force to transform.
	 */
	virtual Vector6d transform(Vector6d force) final;

	friend class SafetyController;
	RobotConstPtr robot_;
	ReferenceFrame frame_;
	Vector6d force_;
};

using ForceGeneratorPtr = std::shared_ptr<ForceGenerator>;
using ForceGeneratorConstPtr = std::shared_ptr<const ForceGenerator>;

} // namespace RSCL
