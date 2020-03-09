/*      File: manipulator_equivalent_mass.h
 *       This file is part of the program open-phri
 *       Program description : OpenPHRI: a generic framework to easily and
 * safely control robots in interactions with humans Copyright (C) 2017 -
 * Benjamin Navarro (LIRMM). All Right reserved.
 *
 *       This software is free software: you can redistribute it and/or modify
 *       it under the terms of the LGPL license as published by
 *       the Free Software Foundation, either version 3
 *       of the License, or (at your option) any later version.
 *       This software is distributed in the hope that it will be useful,
 *       but WITHOUT ANY WARRANTY without even the implied warranty of
 *       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *       LGPL License for more details.
 *
 *       You should have received a copy of the GNU Lesser General Public
 * License version 3 and the General Public License version 3 along with this
 * program. If not, see <http://www.gnu.org/licenses/>.
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

#include <physical_quantities/spatial/type_aliases.h>

namespace phri {

/** @brief A utility to compute a manipulator's equivalent mass.
 *  @details Based on the manipulator's inertia matrix and Jacobian and a set of
 * items that can collide with the its TCP.
 *  @tparam joint_count Number of joints of the manipulator.
 */
class ManipulatorEquivalentMass
    : public ObjectCollection<std::shared_ptr<const Eigen::Vector6d>> {
public:
    /**
     * @brief Construct a manipulator equivalent mass utility given an inertia
     * matrix and a Jacobian. Objects positons must be expressed in the TCP
     * frame.
     * @param inertia_matrix A shared pointer to the inertia matrix.
     * @param jacobian_matrix A shared pointer to the Jacobian matrix.
     */
    ManipulatorEquivalentMass(
        std::shared_ptr<const Eigen::MatrixXd> inertia_matrix,
        std::shared_ptr<const Eigen::MatrixXd> jacobian_matrix);

    /**
     * @brief Construct a manipulator equivalent mass utility given an inertia
     * matrix and a Jacobian. Objects positons must be expressed in the same
     * frame as robot_position.
     * @param inertia_matrix A shared pointer to the inertia matrix.
     * @param jacobian_matrix A shared pointer to the Jacobian matrix.
     * @param robot_position A shared pointer to the robot position in the
     * chosen frame.
     */
    ManipulatorEquivalentMass(
        std::shared_ptr<const Eigen::MatrixXd> inertia_matrix,
        std::shared_ptr<const Eigen::MatrixXd> jacobian_matrix,
        std::shared_ptr<const Eigen::Vector6d> robot_position);

    ~ManipulatorEquivalentMass() = default;

    /**
     * @brief Get the pointer to the equivalent mass.
     * @return A shared pointer to the equivalent mass.
     */
    std::shared_ptr<const double> getEquivalentMass() const;

    /**
     * @brief Compute the equivalent mass based on the inertia, jacobian and the
     * closest item.
     * @return The equivalent mass.
     */
    double compute();

    /**
     * @brief Call operator, shortcut for compute()
     * @return The new output data.
     */
    virtual double operator()() final;

private:
    Eigen::Vector3d closestObjectDirection();

    std::shared_ptr<const Eigen::MatrixXd> inertia_matrix_;
    std::shared_ptr<const Eigen::MatrixXd> jacobian_matrix_;
    std::shared_ptr<const Eigen::Vector6d> robot_position_;
    std::shared_ptr<double> mass_;
};

} // namespace phri
