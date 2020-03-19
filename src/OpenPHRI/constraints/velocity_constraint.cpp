/*      File: velocity_constraint.cpp
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

#include <OpenPHRI/constraints/velocity_constraint.h>
#include <OpenPHRI/utilities/exceptions.h>

#include <iostream>

namespace phri {

VelocityConstraint::VelocityConstraint()
    : maximum_velocity_(scalar::Velocity(0.)) {
}

double VelocityConstraint::compute() {
    double constraint = 1.;
    double v_norm = robot_->control().task().totalVelocity().linear().norm();

    if (v_norm > 0.) {
        constraint = std::abs(getMaximumVelocity().value()) / v_norm;
    }

    return constraint;
}

void VelocityConstraint::setMaximumVelocity(const scalar::Velocity& velocity) {
    maximum_velocity_.ref() = velocity;
}

const scalar::Velocity& VelocityConstraint::getMaximumVelocity() const {
    return maximum_velocity_.cref();
}

} // namespace phri
