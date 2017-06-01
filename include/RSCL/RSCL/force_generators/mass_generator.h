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
 * @file mass_generator.h
 * @author Benjamin Navarro
 * @brief Definition of the MassGenerator class
 * @date April 2017
 * @ingroup RSCL
 */

#pragma once

#include <RSCL/force_generators/force_generator.h>
#include <RSCL/definitions.h>

namespace RSCL {

/** @brief Generates a force as if a virtual mass is attached to the robot.
 */
class MassGenerator : public ForceGenerator {
public:
	/**
	 * @brief Construct a mass generator given a mass and a target acceleration in the TCP frame.
	 * @param mass The virtual mass value.
	 * @param target_acceleration The acceleration target in the TCP frame.
	 */
	MassGenerator(Matrix6dConstPtr mass, Vector6dConstPtr target_acceleration);

	/**
	 * @brief Construct a mass generator given a mass, a target and the robot's acceleration expressed in the same frame.
	 * @param mass The virtual mass value
	 * @param target_acceleration The acceleration target in the chosen frame
	 * @param robot_acceleration The robot's acceleration in the chosen frame
	 */
	MassGenerator(Matrix6dConstPtr mass, Vector6dConstPtr target_acceleration, Vector6dConstPtr robot_acceleration);

	~MassGenerator() = default;

	virtual Vector6d compute() override;

private:
	Matrix6dConstPtr mass_;
	Vector6dConstPtr target_acceleration_;
	Vector6dConstPtr robot_acceleration_;
};

using MassGeneratorPtr = std::shared_ptr<MassGenerator>;
using MassGeneratorConstPtr = std::shared_ptr<const MassGenerator>;

} // namespace RSCL
