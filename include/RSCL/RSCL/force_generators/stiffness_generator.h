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
 * @file stiffness_generator.h
 * @author Benjamin Navarro
 * @brief Definition of the StiffnessGenerator class
 * @date April 2017
 * @ingroup RSCL
 */

#pragma once

#include <RSCL/force_generators/force_generator.h>
#include <RSCL/definitions.h>

namespace RSCL {

/** @brief Generates a force as if a virtual spring is attached to the robot.
 */
class StiffnessGenerator : public ForceGenerator {
public:
	/**
	 * @brief Construct a stiffness generator given a stiffness and a target position in the TCP frame.
	 * @param stiffness The virtual stiffness value.
	 * @param target_position The position target in the TCP frame.
	 */
	StiffnessGenerator(Matrix6dConstPtr stiffness, Vector6dConstPtr target_position);

	/**
	 * @brief Construct a stiffness generator given a stiffness and a target position in the chosen frame.
	 * @param stiffness The virtual stiffness value.
	 * @param target_position The position target in the chosen frame.
	 * @param robot_position The robot position in the chosen frame.
	 */
	StiffnessGenerator(Matrix6dConstPtr stiffness, Vector6dConstPtr target_position, Vector6dConstPtr robot_position);

	~StiffnessGenerator() = default;

	virtual Vector6d compute() override;

private:
	Matrix6dConstPtr stiffness_;
	Vector6dConstPtr target_position_;
	Vector6dConstPtr robot_position_;
};

} // namespace RSCL
