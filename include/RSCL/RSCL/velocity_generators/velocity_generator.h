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
 * @file velocity_generator.h
 * @author Benjamin Navarro
 * @brief Base class definition for velocity generators
 * @date April 2017
 * @ingroup RSCL
 */

#pragma once

#include <memory>

#include <RSCL/definitions.h>
#include <RSCL/robot.h>
#include <RSCL/fwd_decl.h>

namespace RSCL {

/** @brief Base class for all velocity generators.
 *  @details Provides a pure virtual compute method.
 */
class VelocityGenerator {
public:
	~VelocityGenerator() = default;

	/**
	 * @brief Compute the value associated with the velocity generator.
	 * @return The velocity generator's evaluated value.
	 */
	virtual Vector6d compute() = 0;

	/**
	 * @brief Call operator, shortcut for compute().
	 * @return The velocity generator's evaluated value.
	 */
	virtual Vector6d operator()() final;

protected:
	/**
	 * @brief Construct a velocity generator
	 * @param frame The reference frame in which the velocity is expressed.
	 */
	VelocityGenerator(ReferenceFrame frame);

	/**
	 * @brief Transform the given force in the TCP frame, if necessary.
	 * @param velocity The velocity to transform.
	 */
	virtual Vector6d transform(Vector6d velocity) final;

	friend class SafetyController;
	RobotConstPtr robot_;
	ReferenceFrame frame_;
	Vector6d velocity_;
};

using VelocityGeneratorPtr = std::shared_ptr<VelocityGenerator>;
using VelocityGeneratorConstPtr = std::shared_ptr<const VelocityGenerator>;

} // namespace RSCL
