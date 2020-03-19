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
    : maximum_velocities_{vector::dyn::Velocity{}} {
}

double JointVelocityConstraint::compute() {
    if (getMaximumVelocities().size() != robot_->jointCount()) {
        throw std::length_error(OPEN_PHRI_ERROR(
            "The maximum velocity has " +
            std::to_string(getMaximumVelocities().size()) +
            " dofs but the robot has " + std::to_string(robot_->jointCount())));
    }

    double constraint = 1.;
    const auto& joint_vel = robot_->control().joints().totalVelocity();
    const auto& max_joint_vel = getMaximumVelocities();

    for (size_t i = 0; i < joint_vel.size(); ++i) {
        constraint =
            std::min(constraint, max_joint_vel(i) / std::abs(joint_vel(i)));
    }

    return constraint;
}

void JointVelocityConstraint::setMaximumVelocities(
    const vector::dyn::Velocity& velocities) {
    maximum_velocities_.ref() = velocities;
}

const vector::dyn::Velocity&
JointVelocityConstraint::getMaximumVelocities() const {
    return maximum_velocities_.cref();
}

void JointVelocityConstraint::setRobot(Robot const* new_robot) {
    Constraint::setRobot(new_robot);
    if (getMaximumVelocities().size() == 0) {
        maximum_velocities_.ref().resize(robot().jointCount());
    }
}

} // namespace phri
