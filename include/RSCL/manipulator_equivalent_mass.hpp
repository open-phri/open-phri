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
 * @file manipulator_equivalent_mass.hpp
 * @author Benjamin Navarro
 * @brief Definition of the ManipulatorEquivalentMass class
 * @date April 2014
 * @ingroup RSCL
 */

#pragma once

#include <RSCL/definitions.h>
#include <RSCL/object_collection.hpp>

namespace RSCL {

/** @brief A utility to compute a manipulator's equivalent mass.
 *  @details Based on the manipulator's inertia matrix and Jacobian and a set of objects that can collide with the its TCP.
 *  @tparam joint_count Number of joints of the manipulator.
 */
template<int joint_count>
class ManipulatorEquivalentMass : public ObjectCollection<Vector6dConstPtr> {
public:
	using InertiaMatrixType = Eigen::Matrix<double, joint_count, joint_count>;
	using JacobianMatrixType = Eigen::Matrix<double, 6, joint_count>;

	/**
	 * @brief Construct a manipulator equivalent mass utility given an inertia matrix and a Jacobian.
	 * Objects positons must be expressed in the TCP frame.
	 * @param inertia_matrix A shared pointer to the inertia matrix.
	 * @param jacobian_matrix A shared pointer to the Jacobian matrix.
	 */
	ManipulatorEquivalentMass(
		std::shared_ptr<const InertiaMatrixType> inertia_matrix,
		std::shared_ptr<const JacobianMatrixType> jacobian_matrix)
	{
		robot_position_ = std::make_shared<Vector6d>(Vector6d::Zero());
	}

	/**
	 * @brief Construct a manipulator equivalent mass utility given an inertia matrix and a Jacobian.
	 * Objects positons must be expressed in the same frame as robot_position.
	 * @param inertia_matrix A shared pointer to the inertia matrix.
	 * @param jacobian_matrix A shared pointer to the Jacobian matrix.
	 * @param robot_position A shared pointer to the robot position in the chosen frame.
	 */
	ManipulatorEquivalentMass(
		std::shared_ptr<const InertiaMatrixType> inertia_matrix,
		std::shared_ptr<const JacobianMatrixType> jacobian_matrix,
		Vector6dConstPtr robot_position)
	{
		robot_position_ = robot_position;
	}

	~ManipulatorEquivalentMass() = default;

	/**
	 * @brief Get the pointer to the equivalent mass.
	 * @return A shared pointer to the equivalent mass.
	 */
	doubleConstPtr getEquivalentMass() const {
		return mass_;
	}

	/**
	 * @brief Compute the equivalent mass based on the inertia, jacobian and the closest object.
	 * @return The equivalent mass.
	 */
	double compute() {
		const JacobianMatrixType& jac = *jacobian_matrix_;
		const InertiaMatrixType& inertia = *inertia_matrix_;
		Vector3d direction = closestObjectDirection();

		Matrix6d mass_inv = jac * inertia.inverse() * jac.transpose();

		*mass_ = 1. / (direction.transpose() * mass_inv.block<3,3>(0,0) * direction);

		return *mass_;
	}

private:
	Vector3d closestObjectDirection() {
		Vector3d direction = Vector3d::Zero();
		const Vector3d& rob_pos = robot_position_->block<3,1>(0,0);

		double min_dist = std::numeric_limits<double>::infinity();
		for(const auto& object : objects_) {
			Vector6d obj_pos = *object.second;
			Vector3d obj_rob_vec = obj_pos.block<3,1>(0,0) - rob_pos;

			double dist =  obj_rob_vec.norm();
			min_dist = std::min(min_dist, dist);
			if(min_dist == dist) {
				direction = obj_rob_vec.normalized();
			}
		}

		return direction;
	}

	std::shared_ptr<const InertiaMatrixType> inertia_matrix_;
	std::shared_ptr<const JacobianMatrixType> jacobian_matrix_;
	Vector6dConstPtr robot_position_;
	doublePtr mass_;
};

} // namespace RSCL
