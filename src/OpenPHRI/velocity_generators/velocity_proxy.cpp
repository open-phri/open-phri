/*      File: velocity_proxy.cpp
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

#include <OpenPHRI/velocity_generators/velocity_proxy.h>

namespace phri {

VelocityProxy::VelocityProxy(ReferenceFrame frame)
    : VelocityGenerator(frame), external_velocity_(std::make_shared<Twist>()) {
}

VelocityProxy::VelocityProxy(std::shared_ptr<Twist> velocity,
                             ReferenceFrame frame)
    : VelocityGenerator(frame), external_velocity_(velocity) {
}

VelocityProxy::VelocityProxy(Twist& velocity, ReferenceFrame frame)
    : VelocityProxy(std::shared_ptr<Twist>(&velocity, [](auto p) {}), frame) {
}

VelocityProxy::VelocityProxy(const Twist& velocity, ReferenceFrame frame)
    : VelocityProxy(std::make_shared<Twist>(velocity), frame) {
}

VelocityProxy::VelocityProxy(Twist&& velocity, ReferenceFrame frame)
    : VelocityProxy(std::make_shared<Twist>(std::move(velocity)), frame) {
}

void VelocityProxy::update(Twist& velocity) {
    if (generator_) {
        velocity = generator_();
    } else {
        velocity = *external_velocity_;
    }
}

Twist& VelocityProxy::velocity() {
    return *external_velocity_;
}

Twist VelocityProxy::velocity() const {
    return *external_velocity_;
}

std::shared_ptr<Twist> VelocityProxy::velocityPtr() const {
    return external_velocity_;
}

} // namespace phri
