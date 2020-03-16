/*      File: joint_force_proxy.cpp
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

#include <OpenPHRI/joint_force_generators/joint_force_proxy.h>

using namespace phri;

JointForceProxy::JointForceProxy() : external_force_(vector::dyn::Force{}) {
}

JointForceProxy::JointForceProxy(const JointForceProxy::generator& generator)
    : generator_{generator} {
}

void JointForceProxy::update(vector::dyn::Force& joint_force) {
    if (generator_) {
        joint_force = generator_();
    } else {
        joint_force = force();
    }
}

vector::dyn::Force& JointForceProxy::force() {
    return external_force_;
}
const vector::dyn::Force& JointForceProxy::force() const {
    return external_force_;
}

void JointForceProxy::setRobot(Robot const* new_robot) {
    JointForceGenerator::setRobot(new_robot);
    force().resize(robot().jointCount());
    force().setZero();
}