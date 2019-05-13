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

JointVelocityProxy::JointVelocityProxy() : JointVelocityProxy(VectorXd{}) {
}

JointVelocityProxy::JointVelocityProxy(std::shared_ptr<VectorXd> velocity)
    : joint_velocity_(velocity) {
}

JointVelocityProxy::JointVelocityProxy(VectorXd& velocity)
    : JointVelocityProxy(std::shared_ptr<VectorXd>(&velocity, [](auto p) {})) {
}

JointVelocityProxy::JointVelocityProxy(const VectorXd& velocity)
    : JointVelocityProxy(std::make_shared<VectorXd>(velocity)) {
}

JointVelocityProxy::JointVelocityProxy(VectorXd&& velocity)
    : JointVelocityProxy(std::make_shared<VectorXd>(std::move(velocity))) {
}

void JointVelocityProxy::update(VectorXd& velocity) {
    if (generator_) {
        velocity = generator_();
    } else {
        velocity = *joint_velocity_;
    }
}

VectorXd& JointVelocityProxy::velocity() {
    return *joint_velocity_;
}

VectorXd JointVelocityProxy::velocity() const {
    return *joint_velocity_;
}

std::shared_ptr<VectorXd> JointVelocityProxy::velocityPtr() const {
    return joint_velocity_;
}

void JointVelocityProxy::setRobot(Robot const* new_robot) {
    JointVelocityGenerator::setRobot(new_robot);
    velocity().resize(robot().jointCount());
}

} // namespace phri
