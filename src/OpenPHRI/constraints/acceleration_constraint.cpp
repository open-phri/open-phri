/*      File: acceleration_constraint.cpp
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

#include <OpenPHRI/constraints/acceleration_constraint.h>
#include <OpenPHRI/utilities/exceptions.h>

namespace phri {

AccelerationConstraint::AccelerationConstraint()
    : AccelerationConstraint(scalar::Acceleration{0.}) {
}

double AccelerationConstraint::compute() {
    double constraint = 1.;
    double v_norm = robot_->control().task().totalVelocity().linear().norm();

    if (v_norm > 0.) {
        double prev_v_norm =
            robot_->task().command().velocity().linear().norm();
        double vmax = prev_v_norm + std::abs(getMaximumAcceleration().value()) *
                                        robot_->control().timeStep();
        constraint = vmax / v_norm;
    }

    return constraint;
}

void AccelerationConstraint::setMaximumAcceleration(
    const scalar::Acceleration& acceleration) {
    maximum_acceleration_.ref() = acceleration;
}

const scalar::Acceleration&
AccelerationConstraint::getMaximumAcceleration() const {
    return maximum_acceleration_.cref();
}

} // namespace phri
