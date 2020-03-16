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

VelocityProxy::VelocityProxy()
    : external_velocity_(
          spatial::Velocity::Zero(spatial::Frame::Ref(frame()))) {
}

VelocityProxy::VelocityProxy(const generator& generator)
    : generator_(generator) {
}

void VelocityProxy::update(spatial::Velocity& new_velocity) {
    if (generator_) {
        new_velocity = generator_();
    } else {
        new_velocity = velocity();
    }
}

spatial::Velocity& VelocityProxy::velocity() {
    return external_velocity_;
}

const spatial::Velocity& VelocityProxy::velocity() const {
    return external_velocity_;
}

} // namespace phri
