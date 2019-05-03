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
    : maximum_power_(std::make_shared<double>(0.)),
      power_(std::make_shared<double>(0)) {
}

PowerConstraint::PowerConstraint(std::shared_ptr<double> maximum_power)
    : maximum_power_(maximum_power), power_(std::make_shared<double>(0)) {
    if (not maximum_power) {
        throw std::runtime_error(
            OPEN_PHRI_ERROR("You provided an empty shared pointer"));
    }
}

PowerConstraint::PowerConstraint(double& maximum_power)
    : PowerConstraint(std::shared_ptr<double>(&maximum_power, [](auto p) {})) {
}

PowerConstraint::PowerConstraint(const double& maximum_power)
    : PowerConstraint(std::make_shared<double>(maximum_power)) {
}

PowerConstraint::PowerConstraint(double&& maximum_power)
    : PowerConstraint(std::make_shared<double>(std::move(maximum_power))) {
}

double PowerConstraint::compute() {
    double constraint = 1.;
    const Vector3d& velocity = robot_->control.task.total_twist.translation();
    const Vector3d& force = robot_->task.state.wrench.force();
    double power = force.dot(velocity);
    *power_ = power;

    if (power < 0.) {
        constraint = std::abs(*maximum_power_ / power);
    }

    return constraint;
}

double& PowerConstraint::maximumPower() {
    return *maximum_power_;
}

double PowerConstraint::maximumPower() const {
    return *maximum_power_;
}

std::shared_ptr<double> PowerConstraint::maximumPowerPtr() {
    return maximum_power_;
}

double PowerConstraint::power() const {
    return *power_;
}

std::shared_ptr<const double> PowerConstraint::powerPtr() {
    return power_;
}

} // namespace phri
