/*      File: manipulator_equivalent_mass.cpp
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

#include <OpenPHRI/utilities/manipulator_equivalent_mass.h>

using namespace phri;

const scalar::Mass& ManipulatorEquivalentMass::getEquivalentMass() const {
    return mass_;
}

const scalar::Mass& ManipulatorEquivalentMass::compute() {
    const auto& jac = robot_.control().jacobian();
    Eigen::Vector3d direction = closestObjectDirection();

    Eigen::Matrix6d mass_inv = jac * getInertia().inverse() * jac.transpose();

    mass_.value() =
        1. / (direction.transpose() * mass_inv.block<3, 3>(0, 0) * direction);

    return mass_;
}

Eigen::Vector3d ManipulatorEquivalentMass::closestObjectDirection() {
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();

    double min_dist = std::numeric_limits<double>::infinity();
    for (const auto& item : items_) {
        const spatial::Position& obj_pos = item.second;
        PHYSICAL_QUANTITY_CHECK_FRAMES(obj_pos.frame(),
                                       robot_.controlPointFrame());

        double dist = obj_pos.linear().norm();
        min_dist = std::min(min_dist, dist);
        if (min_dist == dist) {
            direction = obj_pos.linear().normalized();
        }
    }

    return direction;
}

const scalar::Mass& ManipulatorEquivalentMass::operator()() {
    return compute();
}

void ManipulatorEquivalentMass::setInertia(const spatial::Mass& inertia) {
    inertia_matrix_.ref() = inertia;
}
const spatial::Mass& ManipulatorEquivalentMass::getInertia() const {
    return inertia_matrix_.cref();
}
