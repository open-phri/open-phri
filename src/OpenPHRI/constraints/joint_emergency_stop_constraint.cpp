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

#include <OpenPHRI/constraints/joint_emergency_stop_constraint.h>
#include <OpenPHRI/utilities/exceptions.h>

namespace phri {

JointEmergencyStopConstraint::JointEmergencyStopConstraint()
    : activation_threshold_(vector::dyn::Force{}),
      deactivation_threshold_(vector::dyn::Force{}),
      previous_constraint_value_(1.) {
}

double JointEmergencyStopConstraint::compute() {
    if (getActivationThreshold().size() != robot_->jointCount()) {
        throw std::length_error(OPEN_PHRI_ERROR(
            "The activation threshold has " +
            std::to_string(getActivationThreshold().size()) +
            " dofs but the robot has " + std::to_string(robot_->jointCount())));
    }

    if (getDeactivationThreshold().size() != robot_->jointCount()) {
        throw std::length_error(OPEN_PHRI_ERROR(
            "The deactivation threshold has " +
            std::to_string(getDeactivationThreshold().size()) +
            " dofs but the robot has " + std::to_string(robot_->jointCount())));
    }

    double constraint = 1.;

    for (size_t i = 0; i < robot_->jointCount(); i++) {
        auto joint_force{std::abs(robot_->joints().state().force()(i))};
        if (joint_force >= getActivationThreshold()(i)) {
            constraint = 0.;
            break;
        }
    }

    if (constraint != 0.) {
        bool all_ok = true;
        for (size_t i = 0; i < robot_->jointCount(); i++) {
            auto joint_force = std::abs(robot_->joints().state().force()(i));
            if (joint_force > getDeactivationThreshold()(i)) {
                all_ok = false;
                break;
            }
        }
        constraint = (all_ok ? 1. : previous_constraint_value_);
    }

    return constraint;
}

void JointEmergencyStopConstraint::setActivationThreshold(
    const vector::dyn::Force& threshold) {
    activation_threshold_.ref() = threshold;
}

const vector::dyn::Force&
JointEmergencyStopConstraint::getActivationThreshold() const {
    return activation_threshold_;
}

void JointEmergencyStopConstraint::setDeactivationThreshold(
    const vector::dyn::Force& threshold) {
    deactivation_threshold_.ref() = threshold;
}

const vector::dyn::Force&
JointEmergencyStopConstraint::getDeactivationThreshold() const {
    return deactivation_threshold_;
}

void JointEmergencyStopConstraint::setRobot(Robot const* new_robot) {
    Constraint::setRobot(new_robot);
    if (getActivationThreshold().size() == 0) {
        activation_threshold_.ref().resize(robot().jointCount());
    }
    if (getDeactivationThreshold().size() == 0) {
        deactivation_threshold_.ref().resize(robot().jointCount());
    }
}

} // namespace phri