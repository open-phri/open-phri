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
 * @file torque_generator.h
 * @author Benjamin Navarro
 * @brief Base class definition for torque generators
 * @date May 2017
 * @ingroup RSCL
 */

#pragma once

#include <RSCL/definitions.h>
#include <RSCL/robot.h>
#include <RSCL/fwd_decl.h>

namespace RSCL {

/** @brief Base class for all torque generators.
 *  @details Provides a pure virtual compute method.
 */
class TorqueGenerator {
public:
	TorqueGenerator() = default;
	~TorqueGenerator() = default;

	/**
	 * @brief Compute the value associated with the torque generator.
	 * @return The torque generator's evaluated value.
	 */
	virtual VectorXd compute() = 0;

	/**
	 * @brief Call operator, shortcut for compute().
	 * @return The torque generator's evaluated value.
	 */
	virtual VectorXd operator()() final;

protected:
	friend class SafetyController;
	RobotConstPtr robot_;
};

using TorqueGeneratorPtr = std::shared_ptr<TorqueGenerator>;
using TorqueGeneratorConstPtr = std::shared_ptr<const TorqueGenerator>;

} // namespace RSCL
