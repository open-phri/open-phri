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

namespace phri {

VelocityConstraint::VelocityConstraint()
    : maximum_velocity_(std::make_shared<scalar::Velocity>(0.)) {
}

VelocityConstraint::VelocityConstraint(
    std::shared_ptr<scalar::Velocity> maximum_velocity)
    : maximum_velocity_(maximum_velocity) {
    if (not maximum_velocity) {
        throw std::runtime_error(
            OPEN_PHRI_ERROR("You provided an empty shared pointer"));
    }
}

VelocityConstraint::VelocityConstraint(scalar::Velocity& maximum_velocity)
    : VelocityConstraint(
          std::shared_ptr<scalar::Velocity>(&maximum_velocity, [](auto p) {})) {
}

VelocityConstraint::VelocityConstraint(const scalar::Velocity& maximum_velocity)
    : VelocityConstraint(std::make_shared<scalar::Velocity>(maximum_velocity)) {
}

VelocityConstraint::VelocityConstraint(scalar::Velocity&& maximum_velocity)
    : VelocityConstraint(
          std::make_shared<scalar::Velocity>(std::move(maximum_velocity))) {
}

double VelocityConstraint::compute() {
    double constraint = 1.;
    double v_norm = robot_->control().task().totalVelocity().linear().norm();

    if (v_norm > 0.) {
        constraint = std::abs(maximum_velocity_->value()) / v_norm;
    }

    return constraint;
}

scalar::Velocity& VelocityConstraint::maximumVelocity() {
    return *maximum_velocity_;
}

scalar::Velocity VelocityConstraint::maximumVelocity() const {
    return *maximum_velocity_;
}

std::shared_ptr<scalar::Velocity>
VelocityConstraint::maximumVelocityPtr() const {
    return maximum_velocity_;
}

} // namespace phri
