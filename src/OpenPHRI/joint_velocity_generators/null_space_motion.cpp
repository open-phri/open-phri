/*      File: null_space_motion.cpp
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

#include <OpenPHRI/joint_velocity_generators/null_space_motion.h>
#include <iostream>

namespace phri {

NullSpaceMotion::NullSpaceMotion() : NullSpaceMotion(vector::dyn::Velocity{}) {
}

void NullSpaceMotion::update(vector::dyn::Velocity& joint_velocity) {
    null_space_projector_ = identity_ - robot().control().jacobianInverse() *
                                            robot().control().jacobian();
    joint_velocity.value() = null_space_projector_ * getVelocity();
}

void NullSpaceMotion::setVelocity(const vector::dyn::Velocity& velocity) {
    joint_velocity_.ref() = velocity;
}

const vector::dyn::Velocity& NullSpaceMotion::getVelocity() const {
    return joint_velocity_.cref();
}

void NullSpaceMotion::setRobot(Robot const* new_robot) {
    JointVelocityGenerator::setRobot(new_robot);
    auto dofs = robot().jointCount();

    if (getVelocity().size() == 0) {
        joint_velocity_.ref().resize(dofs);
        joint_velocity_.ref().setZero();
    }

    null_space_projector_.resize(dofs, dofs);
    identity_.resize(dofs, dofs);
    identity_.setIdentity();
}

} // namespace phri
