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
 * @file joint_velocity_generator.h
 * @author Benjamin Navarro
 * @brief Base class definition for joint_velocity generators
 * @date April 2017
 * @ingroup RSCL
 */

#pragma once

#include <memory>

#include <RSCL/definitions.h>

namespace RSCL {

/** @brief Base class for all joint_velocity generators.
 *  @details Provides a pure virtual compute method.
 */
class JointVelocityGenerator {
public:
	JointVelocityGenerator() = default;
	~JointVelocityGenerator() = default;

	/**
	 * @brief Compute the value associated with the joint_velocity generator.
	 * @return The joint_velocity generator's evaluated value.
	 */
	virtual VectorXd compute() = 0;
};

using JointVelocityGeneratorPtr = std::shared_ptr<JointVelocityGenerator>;
using JointVelocityGeneratorConstPtr = std::shared_ptr<const JointVelocityGenerator>;

} // namespace RSCL
