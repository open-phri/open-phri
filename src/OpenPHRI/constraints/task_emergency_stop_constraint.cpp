/*      File: stop_constraint.cpp
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

#include <OpenPHRI/constraints/task_emergency_stop_constraint.h>
#include <OpenPHRI/utilities/exceptions.h>

namespace phri {

TaskEmergencyStopConstraint::TaskEmergencyStopConstraint()
    : activation_threshold_(std::make_shared<scalar::Force>(0.)),
      deactivation_threshold_(std::make_shared<scalar::Force>(0.)),
      previous_constraint_value_(1.) {
}

TaskEmergencyStopConstraint::TaskEmergencyStopConstraint(
    std::shared_ptr<scalar::Force> activation_threshold,
    std::shared_ptr<scalar::Force> deactivation_threshold)
    : activation_threshold_(activation_threshold),
      deactivation_threshold_(deactivation_threshold),
      previous_constraint_value_(1.) {
    if (not activation_threshold or not deactivation_threshold) {
        throw std::runtime_error(
            OPEN_PHRI_ERROR("You provided an empty shared pointer"));
    }
}

TaskEmergencyStopConstraint::TaskEmergencyStopConstraint(
    scalar::Force& activation_threshold, scalar::Force& deactivation_threshold)
    : TaskEmergencyStopConstraint(
          std::shared_ptr<scalar::Force>(&activation_threshold, [](auto p) {}),
          std::shared_ptr<scalar::Force>(&deactivation_threshold,
                                         [](auto p) {})) {
}

TaskEmergencyStopConstraint::TaskEmergencyStopConstraint(
    const scalar::Force& activation_threshold,
    const scalar::Force& deactivation_threshold)
    : TaskEmergencyStopConstraint(
          std::make_shared<scalar::Force>(activation_threshold),
          std::make_shared<scalar::Force>(deactivation_threshold)) {
}

TaskEmergencyStopConstraint::TaskEmergencyStopConstraint(
    scalar::Force&& activation_threshold,
    scalar::Force&& deactivation_threshold)
    : TaskEmergencyStopConstraint(
          std::make_shared<scalar::Force>(std::move(activation_threshold)),
          std::make_shared<scalar::Force>(std::move(deactivation_threshold))) {
}

double TaskEmergencyStopConstraint::compute() {
    double constraint;

    double norm = robot_->task().state().force().linear().norm();

    if (norm >= activationThreshold().value()) {
        constraint = 0.;
    } else if (norm <= deactivationThreshold().value()) {
        constraint = 1.;
    } else {
        constraint = previous_constraint_value_;
    }

    previous_constraint_value_ = constraint;

    return constraint;
}

scalar::Force& TaskEmergencyStopConstraint::activationThreshold() {
    return *activation_threshold_;
}

scalar::Force TaskEmergencyStopConstraint::activationThreshold() const {
    return *activation_threshold_;
}

std::shared_ptr<scalar::Force>
TaskEmergencyStopConstraint::activationThresholdPtr() const {
    return activation_threshold_;
}

scalar::Force& TaskEmergencyStopConstraint::deactivationThreshold() {
    return *deactivation_threshold_;
}

scalar::Force TaskEmergencyStopConstraint::deactivationThreshold() const {
    return *deactivation_threshold_;
}

std::shared_ptr<scalar::Force>
TaskEmergencyStopConstraint::deactivationThresholdPtr() const {
    return deactivation_threshold_;
}

} // namespace phri