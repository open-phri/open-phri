/*      File: joint_velocity_constraint.cpp
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

#include <OpenPHRI/constraints/joint_velocity_constraint.h>
#include <OpenPHRI/utilities/exceptions.h>

namespace phri {

JointVelocityConstraint::JointVelocityConstraint()
    : maximum_velocities_(std::make_shared<VectorXd>()) {
}

JointVelocityConstraint::JointVelocityConstraint(
    std::shared_ptr<VectorXd> maximum_velocities)
    : maximum_velocities_(maximum_velocities) {
    if (not maximum_velocities) {
        throw std::runtime_error(
            OPEN_PHRI_ERROR("You provided an empty shared pointer"));
    }
}

JointVelocityConstraint::JointVelocityConstraint(VectorXd& maximum_velocities)
    : JointVelocityConstraint(
          std::shared_ptr<VectorXd>(&maximum_velocities, [](auto p) {})) {
}

JointVelocityConstraint::JointVelocityConstraint(
    const VectorXd& maximum_velocities)
    : JointVelocityConstraint(std::make_shared<VectorXd>(maximum_velocities)) {
}

JointVelocityConstraint::JointVelocityConstraint(VectorXd&& maximum_velocities)
    : JointVelocityConstraint(
          std::make_shared<VectorXd>(std::move(maximum_velocities))) {
}

double JointVelocityConstraint::compute() {
    if (maximumVelocities().size() != robot_->jointCount()) {
        throw std::length_error(OPEN_PHRI_ERROR(
            "The maximum velocity has " +
            std::to_string(maximumVelocities().size()) +
            " dofs but the robot has " + std::to_string(robot_->jointCount())));
    }

    double constraint = 1.;
    const auto& joint_vel = robot_->control.joints.total_velocity;
    const auto& max_joint_vel = *maximum_velocities_;

    for (size_t i = 0; i < joint_vel.size(); ++i) {
        constraint =
            std::min(constraint, max_joint_vel(i) / std::abs(joint_vel(i)));
    }

    return constraint;
}

VectorXd& JointVelocityConstraint::maximumVelocities() {
    return *maximum_velocities_;
}

const VectorXd& JointVelocityConstraint::maximumVelocities() const {
    return *maximum_velocities_;
}

std::shared_ptr<VectorXd>
JointVelocityConstraint::maximumVelocitiesPtr() const {
    return maximum_velocities_;
}

void JointVelocityConstraint::setRobot(Robot const* new_robot) {
    Constraint::setRobot(new_robot);
    maximumVelocities().resize(robot().jointCount());
}

} // namespace phri
