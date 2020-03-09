/*      File: force_proxy.cpp
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

#include <OpenPHRI/force_generators/force_proxy.h>

namespace phri {

// ForceProxy::ForceProxy(spatial::Frame frame)
//     : ForceProxy(frame, spatial::Force{frame}) {
// }

ForceProxy::ForceProxy(std::shared_ptr<spatial::Force> velocity)
    : external_force_(velocity) {
}

ForceProxy::ForceProxy(spatial::Force& velocity)
    : ForceProxy(std::shared_ptr<spatial::Force>(&velocity, [](auto p) {})) {
}

ForceProxy::ForceProxy(const spatial::Force& velocity)
    : ForceProxy(std::make_shared<spatial::Force>(velocity)) {
}

ForceProxy::ForceProxy(spatial::Force&& velocity)
    : ForceProxy(std::make_shared<spatial::Force>(std::move(velocity))) {
}

void ForceProxy::update(spatial::Force& velocity) {
    if (generator_) {
        velocity = generator_();
    } else {
        velocity = *external_force_;
    }
}

spatial::Force& ForceProxy::force() {
    return *external_force_;
}

const spatial::Force& ForceProxy::force() const {
    return *external_force_;
}

std::shared_ptr<spatial::Force> ForceProxy::forcePtr() const {
    return external_force_;
}

} // namespace phri