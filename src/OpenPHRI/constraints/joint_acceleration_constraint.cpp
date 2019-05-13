/*      File: joint_acceleration_constraint.cpp
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

#include <OpenPHRI/constraints/joint_acceleration_constraint.h>
#include <OpenPHRI/utilities/exceptions.h>

namespace phri {

JointAccelerationConstraint::JointAccelerationConstraint()
    : JointAccelerationConstraint(VectorXd{}) {
}

JointAccelerationConstraint::JointAccelerationConstraint(
    std::shared_ptr<VectorXd> maximum_acceleration)
    : maximum_acceleration_(maximum_acceleration) {
    if (not maximum_acceleration) {
        throw std::runtime_error(
            OPEN_PHRI_ERROR("You provided an empty shared pointer"));
    }
}

JointAccelerationConstraint::JointAccelerationConstraint(
    VectorXd& maximum_acceleration)
    : JointAccelerationConstraint(
          std::shared_ptr<VectorXd>(&maximum_acceleration, [](auto p) {})) {
}

JointAccelerationConstraint::JointAccelerationConstraint(
    const VectorXd& maximum_acceleration)
    : JointAccelerationConstraint(
          std::make_shared<VectorXd>(maximum_acceleration)) {
}

JointAccelerationConstraint::JointAccelerationConstraint(
    VectorXd&& maximum_acceleration)
    : JointAccelerationConstraint(
          std::make_shared<VectorXd>(std::move(maximum_acceleration))) {
}

double JointAccelerationConstraint::compute() {
    double constraint = 1.;
    const auto& joint_vel = robot_->control.joints.total_velocity;
    const auto& max_joint_acc = *maximum_acceleration_;
    const auto& prev_joint_vel = robot_->joints.command.velocity;

    for (size_t i = 0; i < joint_vel.size(); ++i) {
        if (joint_vel(i) < 1e-6) {
            continue;
        }
        constraint = std::min(constraint,
                              (std::abs(prev_joint_vel(i)) +
                               max_joint_acc(i) * robot().control.time_step) /
                                  std::abs(joint_vel(i)));
    }

    return constraint;
}

VectorXd& JointAccelerationConstraint::maximumAcceleration() {
    return *maximum_acceleration_;
}

VectorXd JointAccelerationConstraint::maximumAcceleration() const {
    return *maximum_acceleration_;
}

std::shared_ptr<VectorXd>
JointAccelerationConstraint::maximumAccelerationPtr() const {
    return maximum_acceleration_;
}

void JointAccelerationConstraint::setRobot(Robot const* new_robot) {
    Constraint::setRobot(new_robot);
    maximumAcceleration().resize(robot().jointCount());
}

} // namespace phri
