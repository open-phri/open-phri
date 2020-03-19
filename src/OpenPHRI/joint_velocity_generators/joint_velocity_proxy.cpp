/*      File: joint_velocity_proxy.cpp
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

#include <OpenPHRI/joint_velocity_generators/joint_velocity_proxy.h>

namespace phri {

JointVelocityProxy::JointVelocityProxy()
    : JointVelocityProxy{vector::dyn::Velocity{}} {
}

void JointVelocityProxy::update(vector::dyn::Velocity& new_velocity) {
    if (generator_) {
        new_velocity = generator_();
    } else {
        new_velocity = getVelocity();
    }
}

void JointVelocityProxy::setVelocity(const vector::dyn::Velocity& velocity) {
    external_velocity_.ref() = velocity;
}

const vector::dyn::Velocity& JointVelocityProxy::getVelocity() const {
    return external_velocity_.cref();
}

void JointVelocityProxy::setRobot(Robot const* new_robot) {
    JointVelocityGenerator::setRobot(new_robot);
    if (getVelocity().size() == 0) {
        external_velocity_.ref().resize(robot().jointCount());
        external_velocity_.ref().setZero();
    }
}

} // namespace phri
