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
    : activation_threshold_(std::make_shared<VectorXd>()),
      deactivation_threshold_(std::make_shared<VectorXd>()),
      previous_constraint_value_(1.) {
}

JointEmergencyStopConstraint::JointEmergencyStopConstraint(
    std::shared_ptr<VectorXd> activation_threshold,
    std::shared_ptr<VectorXd> deactivation_threshold)
    : activation_threshold_(activation_threshold),
      deactivation_threshold_(deactivation_threshold),
      previous_constraint_value_(1.) {
    if (not activation_threshold or not deactivation_threshold) {
        throw std::runtime_error(
            OPEN_PHRI_ERROR("You provided an empty shared pointer"));
    }
}

JointEmergencyStopConstraint::JointEmergencyStopConstraint(
    VectorXd& activation_threshold, VectorXd& deactivation_threshold)
    : JointEmergencyStopConstraint(
          std::shared_ptr<VectorXd>(&activation_threshold, [](auto p) {}),
          std::shared_ptr<VectorXd>(&deactivation_threshold, [](auto p) {})) {
}

JointEmergencyStopConstraint::JointEmergencyStopConstraint(
    const VectorXd& activation_threshold,
    const VectorXd& deactivation_threshold)
    : JointEmergencyStopConstraint(
          std::make_shared<VectorXd>(activation_threshold),
          std::make_shared<VectorXd>(deactivation_threshold)) {
}

JointEmergencyStopConstraint::JointEmergencyStopConstraint(
    VectorXd&& activation_threshold, VectorXd&& deactivation_threshold)
    : JointEmergencyStopConstraint(
          std::make_shared<VectorXd>(std::move(activation_threshold)),
          std::make_shared<VectorXd>(std::move(deactivation_threshold))) {
}

double JointEmergencyStopConstraint::compute() {
    if (activationThreshold().size() != robot_->jointCount()) {
        throw std::length_error(OPEN_PHRI_ERROR(
            "The activation threshold has " +
            std::to_string(activationThreshold().size()) +
            " dofs but the robot has " + std::to_string(robot_->jointCount())));
    }

    if (deactivationThreshold().size() != robot_->jointCount()) {
        throw std::length_error(OPEN_PHRI_ERROR(
            "The deactivation threshold has " +
            std::to_string(deactivationThreshold().size()) +
            " dofs but the robot has " + std::to_string(robot_->jointCount())));
    }

    double constraint = 1.;

    for (size_t i = 0; i < robot_->jointCount(); i++) {
        auto joint_force{std::abs(robot_->joints.state.force(i))};
        if (joint_force >= activationThreshold()(i)) {
            constraint = 0.;
            break;
        }
    }

    if (constraint != 0.) {
        bool all_ok = true;
        for (size_t i = 0; i < robot_->jointCount(); i++) {
            auto joint_force = std::abs(robot_->joints.state.force(i));
            if (joint_force > deactivationThreshold()(i)) {
                all_ok = false;
                break;
            }
        }
        constraint = (all_ok ? 1. : previous_constraint_value_);
    }

    return constraint;
}

VectorXd& JointEmergencyStopConstraint::activationThreshold() {
    return *activation_threshold_;
}

const VectorXd& JointEmergencyStopConstraint::activationThreshold() const {
    return *activation_threshold_;
}

std::shared_ptr<VectorXd>
JointEmergencyStopConstraint::activationThresholdPtr() {
    return activation_threshold_;
}

VectorXd& JointEmergencyStopConstraint::deactivationThreshold() {
    return *deactivation_threshold_;
}

const VectorXd& JointEmergencyStopConstraint::deactivationThreshold() const {
    return *deactivation_threshold_;
}

std::shared_ptr<VectorXd>
JointEmergencyStopConstraint::deactivationThresholdPtr() {
    return deactivation_threshold_;
}

} // namespace phri