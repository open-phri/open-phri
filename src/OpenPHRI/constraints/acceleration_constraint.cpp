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
    : maximum_acceleration_(std::make_shared<double>(0.)) {
}

AccelerationConstraint::AccelerationConstraint(
    std::shared_ptr<double> maximum_acceleration)
    : maximum_acceleration_(maximum_acceleration) {
    if (not maximum_acceleration) {
        throw std::runtime_error(
            OPEN_PHRI_ERROR("You provided an empty shared pointer"));
    }
}

AccelerationConstraint::AccelerationConstraint(double& maximum_acceleration)
    : maximum_acceleration_(
          std::shared_ptr<double>(&maximum_acceleration, [](auto p) {})) {
}

AccelerationConstraint::AccelerationConstraint(
    const double& maximum_acceleration)
    : maximum_acceleration_(std::make_shared<double>(maximum_acceleration)) {
}

AccelerationConstraint::AccelerationConstraint(double&& maximum_acceleration)
    : maximum_acceleration_(
          std::make_shared<double>(std::move(maximum_acceleration))) {
}

double AccelerationConstraint::compute() {
    double constraint = 1.;
    double v_norm = robot_->control.task.total_twist.translation().norm();

    if (v_norm > 0.) {
        double prev_v_norm = robot_->task.command.twist.translation().norm();
        double vmax = prev_v_norm + std::abs(*maximum_acceleration_) *
                                        robot_->control.time_step;
        constraint = vmax / v_norm;
    }

    return constraint;
}

double& AccelerationConstraint::maximumAcceleration() {
    return *maximum_acceleration_;
}

double AccelerationConstraint::maximumAcceleration() const {
    return *maximum_acceleration_;
}

std::shared_ptr<double> AccelerationConstraint::maximumAccelerationPtr() const {
    return maximum_acceleration_;
}

} // namespace phri
