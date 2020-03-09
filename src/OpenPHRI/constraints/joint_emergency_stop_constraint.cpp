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
    : activation_threshold_(std::make_shared<Eigen::VectorXd>()),
      deactivation_threshold_(std::make_shared<Eigen::VectorXd>()),
      previous_constraint_value_(1.) {
}

JointEmergencyStopConstraint::JointEmergencyStopConstraint(
    std::shared_ptr<Eigen::VectorXd> activation_threshold,
    std::shared_ptr<Eigen::VectorXd> deactivation_threshold)
    : activation_threshold_(activation_threshold),
      deactivation_threshold_(deactivation_threshold),
      previous_constraint_value_(1.) {
    if (not activation_threshold or not deactivation_threshold) {
        throw std::runtime_error(
            OPEN_PHRI_ERROR("You provided an empty shared pointer"));
    }
}

JointEmergencyStopConstraint::JointEmergencyStopConstraint(
    Eigen::VectorXd& activation_threshold,
    Eigen::VectorXd& deactivation_threshold)
    : JointEmergencyStopConstraint(
          std::shared_ptr<Eigen::VectorXd>(&activation_threshold,
                                           [](auto p) {}),
          std::shared_ptr<Eigen::VectorXd>(&deactivation_threshold,
                                           [](auto p) {})) {
}

JointEmergencyStopConstraint::JointEmergencyStopConstraint(
    const Eigen::VectorXd& activation_threshold,
    const Eigen::VectorXd& deactivation_threshold)
    : JointEmergencyStopConstraint(
          std::make_shared<Eigen::VectorXd>(activation_threshold),
          std::make_shared<Eigen::VectorXd>(deactivation_threshold)) {
}

JointEmergencyStopConstraint::JointEmergencyStopConstraint(
    Eigen::VectorXd&& activation_threshold,
    Eigen::VectorXd&& deactivation_threshold)
    : JointEmergencyStopConstraint(
          std::make_shared<Eigen::VectorXd>(std::move(activation_threshold)),
          std::make_shared<Eigen::VectorXd>(
              std::move(deactivation_threshold))) {
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
        auto joint_force{std::abs(robot_->joints().state().force()(i))};
        if (joint_force >= activationThreshold()(i)) {
            constraint = 0.;
            break;
        }
    }

    if (constraint != 0.) {
        bool all_ok = true;
        for (size_t i = 0; i < robot_->jointCount(); i++) {
            auto joint_force = std::abs(robot_->joints().state().force()(i));
            if (joint_force > deactivationThreshold()(i)) {
                all_ok = false;
                break;
            }
        }
        constraint = (all_ok ? 1. : previous_constraint_value_);
    }

    return constraint;
}

Eigen::VectorXd& JointEmergencyStopConstraint::activationThreshold() {
    return *activation_threshold_;
}

const Eigen::VectorXd&
JointEmergencyStopConstraint::activationThreshold() const {
    return *activation_threshold_;
}

std::shared_ptr<Eigen::VectorXd>
JointEmergencyStopConstraint::activationThresholdPtr() const {
    return activation_threshold_;
}

Eigen::VectorXd& JointEmergencyStopConstraint::deactivationThreshold() {
    return *deactivation_threshold_;
}

const Eigen::VectorXd&
JointEmergencyStopConstraint::deactivationThreshold() const {
    return *deactivation_threshold_;
}

std::shared_ptr<Eigen::VectorXd>
JointEmergencyStopConstraint::deactivationThresholdPtr() const {
    return deactivation_threshold_;
}

void JointEmergencyStopConstraint::setRobot(Robot const* new_robot) {
    Constraint::setRobot(new_robot);
    activationThreshold().resize(robot().jointCount());
    deactivationThreshold().resize(robot().jointCount());
}

} // namespace phri