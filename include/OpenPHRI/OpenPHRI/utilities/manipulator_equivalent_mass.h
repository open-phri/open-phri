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
 * @file manipulator_equivalent_mass.hpp
 * @author Benjamin Navarro
 * @brief Definition of the ManipulatorEquivalentMass class
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/utilities/object_collection.hpp>

namespace phri {

/** @brief A utility to compute a manipulator's equivalent mass.
 *  @details Based on the manipulator's inertia matrix and Jacobian and a set of items that can collide with the its TCP.
 *  @tparam joint_count Number of joints of the manipulator.
 */
class ManipulatorEquivalentMass : public ObjectCollection<Vector6dConstPtr> {
public:
	/**
	 * @brief Construct a manipulator equivalent mass utility given an inertia matrix and a Jacobian.
	 * Objects positons must be expressed in the TCP frame.
	 * @param inertia_matrix A shared pointer to the inertia matrix.
	 * @param jacobian_matrix A shared pointer to the Jacobian matrix.
	 */
	ManipulatorEquivalentMass(
		MatrixXdConstPtr inertia_matrix,
		MatrixXdConstPtr jacobian_matrix);

	/**
	 * @brief Construct a manipulator equivalent mass utility given an inertia matrix and a Jacobian.
	 * Objects positons must be expressed in the same frame as robot_position.
	 * @param inertia_matrix A shared pointer to the inertia matrix.
	 * @param jacobian_matrix A shared pointer to the Jacobian matrix.
	 * @param robot_position A shared pointer to the robot position in the chosen frame.
	 */
	ManipulatorEquivalentMass(
		MatrixXdConstPtr inertia_matrix,
		MatrixXdConstPtr jacobian_matrix,
		Vector6dConstPtr robot_position);

	~ManipulatorEquivalentMass() = default;

	/**
	 * @brief Get the pointer to the equivalent mass.
	 * @return A shared pointer to the equivalent mass.
	 */
	doubleConstPtr getEquivalentMass() const;

	/**
	 * @brief Compute the equivalent mass based on the inertia, jacobian and the closest item.
	 * @return The equivalent mass.
	 */
	double compute();

	/**
	 * @brief Call operator, shortcut for compute()
	 * @return The new output data.
	 */
	virtual double operator()() final;

private:
	Vector3d closestObjectDirection();

	MatrixXdConstPtr inertia_matrix_;
	MatrixXdConstPtr jacobian_matrix_;
	Vector6dConstPtr robot_position_;
	doublePtr mass_;
};

} // namespace phri
