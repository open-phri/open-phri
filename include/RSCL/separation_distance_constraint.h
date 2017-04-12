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
 * @file separation_distance_constraint.h
 * @author Benjamin Navarro
 * @brief Definition of the SeparationDistanceConstraint class
 * @date April 2014
 * @ingroup RSCL
 */

#pragma once

#include <RSCL/definitions.h>
#include <RSCL/constraint.h>
#include <RSCL/interpolator.h>
#include <RSCL/object_collection.hpp>
#include <map>

namespace RSCL {

/** @brief A meta-constraint to adapt a given constraint depending on the distance to the closest object.
 *  @details You have to provide preconfigured constraint and interpolator.
 *  The interpolator input is set to the serapration distance.
 */
class SeparationDistanceConstraint : public Constraint, public ObjectCollection<Vector6dConstPtr> {
public:
	/***		Constructor & destructor		***/

	/**
	 * @brief Construct a separaration distance constraint with a given constraint and interpolator.
	 * Objects position must be expressed in the TCP frame.
	 * @param constraint The constraint to wrap.
	 * @param interpolator The interpolator used to tune the constraint.
	 */
	SeparationDistanceConstraint(
		ConstraintPtr constraint,
		InterpolatorPtr interpolator);

	/**
	 * @brief Construct a separaration distance constraint with a given constraint, interpolator and robot positon.
	 * Objects position must be expressed in the same frame as robot_position.
	 * @param constraint The constraint to wrap.
	 * @param interpolator The interpolator used to tune the constraint.
	 * @param robot_position The positon of the robot in the same frame as the objects.
	 */
	SeparationDistanceConstraint(
		ConstraintPtr constraint,
		InterpolatorPtr interpolator,
		Vector6dConstPtr robot_position);

	virtual ~SeparationDistanceConstraint() = default;

	/***		Algorithm        ***/
	virtual double compute() override;

	/**
	 * @brief Retrieve the separation shared pointer.
	 * @return The shared pointer to the separation power.
	 */
	doubleConstPtr getSeparationDistance() const;

private:
	double closestObjectDistance();

	ConstraintPtr constraint_;
	InterpolatorPtr interpolator_;
	Vector6dConstPtr robot_position_;
	doublePtr separation_distance_;
};

using SeparationDistanceConstraintPtr = std::shared_ptr<SeparationDistanceConstraint>;
using SeparationDistanceConstraintConstPtr = std::shared_ptr<const SeparationDistanceConstraint>;

} // namespace RSCL
