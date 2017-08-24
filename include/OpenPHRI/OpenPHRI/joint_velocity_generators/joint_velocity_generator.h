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
 * @file joint_velocity_generator.h
 * @author Benjamin Navarro
 * @brief Base class definition for joint_velocity generators
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <memory>

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/robot.h>
#include <OpenPHRI/fwd_decl.h>

namespace phri {

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

	/**
	 * @brief Call operator, shortcut for compute().
	 * @return The joint_velocity generator's evaluated value.
	 */
	virtual VectorXd operator()() final;

protected:
	friend class SafetyController;
	RobotConstPtr robot_;
};

using JointVelocityGeneratorPtr = std::shared_ptr<JointVelocityGenerator>;
using JointVelocityGeneratorConstPtr = std::shared_ptr<const JointVelocityGenerator>;

} // namespace phri
