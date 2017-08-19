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
 * @file stiffness_generator.h
 * @author Benjamin Navarro
 * @brief Definition of the StiffnessGenerator class
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/force_generators/force_generator.h>
#include <OpenPHRI/definitions.h>

namespace OpenPHRI {

/** @brief Generates a force as if a virtual spring is attached to the robot.
 */
class StiffnessGenerator : public ForceGenerator {
public:
	/**
	 * @brief Construct a stiffness generator given a stiffness and a target position.
	 * @param stiffness The virtual stiffness value.
	 * @param target_position The position target.
	 * @param stiffness_frame The frame in which the stiffness is expressed.
	 * @param target_position_frame The frame in which the position target is expressed.
	 */
	StiffnessGenerator(
		Matrix6dConstPtr stiffness,
		Vector6dConstPtr target_position,
		ReferenceFrame stiffness_frame = ReferenceFrame::TCP,
		ReferenceFrame target_position_frame = ReferenceFrame::TCP);

	~StiffnessGenerator() = default;

	virtual Vector6d compute() override;

private:
	Matrix6dConstPtr stiffness_;
	Vector6dConstPtr target_position_;
	ReferenceFrame stiffness_frame_;
	ReferenceFrame target_position_frame_;
};

using StiffnessGeneratorPtr = std::shared_ptr<StiffnessGenerator>;
using StiffnessGeneratorConstPtr = std::shared_ptr<const StiffnessGenerator>;

} // namespace OpenPHRI
