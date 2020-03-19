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
#include <OpenPHRI/robot.h>
#include <OpenPHRI/utilities/object_collection.hpp>
#include <OpenPHRI/detail/universal_wrapper.hpp>

#include <physical_quantities/spatial/type_aliases.h>
#include <physical_quantities/spatial/position.h>
#include <physical_quantities/spatial/impedance/mass.h>
#include <physical_quantities/scalar/mass.h>

namespace phri {

/** @brief A utility to compute a manipulator's equivalent mass.
 *  @details Based on the manipulator's inertia matrix and Jacobian and a set of
 * items that can collide with the its TCP.
 *  @tparam joint_count Number of joints of the manipulator.
 */
class ManipulatorEquivalentMass : public ObjectCollection<spatial::Position> {
public:
    /**
     * @brief Construct a manipulator equivalent mass utility given an inertia
     * matrix and a Jacobian. Objects positons must be expressed in the TCP
     * frame.
     * @param inertia_matrix A shared pointer to the inertia matrix.
     */
    template <typename InertiaT>
    ManipulatorEquivalentMass(const Robot& robot, InertiaT&& inertia_matrix)
        : robot_{robot},
          inertia_matrix_{std::forward<InertiaT>(inertia_matrix)} {
    }

    /**
     * @brief Read only access to the equivalent mass.
     * @return const double& A reference to the equivalent mass.
     */
    const scalar::Mass& getEquivalentMass() const;

    /**
     * @brief Compute the equivalent mass based on the inertia, jacobian and the
     * closest item.
     * @return The equivalent mass.
     */
    const scalar::Mass& compute();

    /**
     * @brief Call operator, shortcut for compute()
     * @return The new output data.
     */
    const scalar::Mass& operator()();

    void setInertia(const spatial::Mass& inertia);
    const spatial::Mass& getInertia() const;

private:
    Eigen::Vector3d closestObjectDirection();

    const Robot& robot_;
    detail::UniversalWrapper<spatial::Mass> inertia_matrix_;
    scalar::Mass mass_;
};

} // namespace phri
