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
    : JointAccelerationConstraint{vector::dyn::Acceleration{}} {
}

double JointAccelerationConstraint::compute() {
    double constraint = 1.;
    const auto& joint_vel = robot_->control().joints().totalVelocity();
    const auto& max_joint_acc = getMaximumAcceleration().value();
    const auto& prev_joint_vel = robot_->joints().command().velocity();

    for (size_t i = 0; i < joint_vel.size(); ++i) {
        if (joint_vel(i) < 1e-6) {
            continue;
        }
        constraint = std::min(
            constraint, (std::abs(prev_joint_vel(i)) +
                         max_joint_acc(i) * robot().control().timeStep()) /
                            std::abs(joint_vel(i)));
    }

    return constraint;
}

void JointAccelerationConstraint::setMaximumAcceleration(
    const vector::dyn::Acceleration& acceleration) {
    maximum_acceleration_.ref() = acceleration;
}

const vector::dyn::Acceleration&
JointAccelerationConstraint::getMaximumAcceleration() const {
    return maximum_acceleration_.cref();
}

void JointAccelerationConstraint::setRobot(Robot const* new_robot) {
    Constraint::setRobot(new_robot);
    if (getMaximumAcceleration().size() == 0) {
        maximum_acceleration_.ref().resize(robot().jointCount());
    }
}

} // namespace phri
