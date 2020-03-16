/*      File: power_constraint.cpp
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

#include <OpenPHRI/constraints/power_constraint.h>
#include <OpenPHRI/utilities/exceptions.h>

namespace phri {

PowerConstraint::PowerConstraint()
    : maximum_power_{scalar::Power{0.}}, power_{0.} {
}

double PowerConstraint::compute() {
    double constraint = 1.;
    const auto& velocity = robot_->control().task().totalVelocity().linear();
    const auto& force = robot_->task().state().force().linear();
    scalar::Power power = spatial::dot(force, velocity);
    power_ = power;

    if (power < scalar::Power{0.}) {
        constraint = std::abs(maximumPower().value() / power.value());
    }

    return constraint;
}

scalar::Power& PowerConstraint::maximumPower() {
    return maximum_power_;
}

const scalar::Power& PowerConstraint::maximumPower() const {
    return maximum_power_;
}

const scalar::Power& PowerConstraint::power() const {
    return power_;
}

} // namespace phri
