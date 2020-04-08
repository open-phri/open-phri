/*      File: mass_generator.cpp
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

#include <OpenPHRI/force_generators/mass_generator.h>

#include <physical_quantities/spatial/impedance/mass.hpp>

namespace phri {

MassGenerator::MassGenerator()
    : mass_{spatial::Mass::Zero(spatial::Frame::Ref(frame()))},
      target_acceleration_{
          spatial::Acceleration::Zero(spatial::Frame::Ref(frame()))} {
}

void MassGenerator::update(spatial::Force& force) {
    auto to_cp_frame = [this](const auto& value) {
        if (value.frame() != robot().controlPointFrame()) {
            return robot().control().transformation().inverse() * value;
        } else {
            return value;
        }
    };

    auto current_accel_cp = to_cp_frame(robot().task().state().acceleration());
    auto target_accel_cp = to_cp_frame(getTargetAcceleration());
    auto mass_cp = to_cp_frame(getMass());

    force = mass_cp * (target_accel_cp - current_accel_cp);
}

void MassGenerator::setMass(const spatial::Mass& mass) {
    mass_.ref() = mass;
}

const spatial::Mass& MassGenerator::getMass() const {
    return mass_.cref();
}

void MassGenerator::setTargetAcceleration(
    const spatial::Acceleration& acceleration) {
    target_acceleration_.ref() = acceleration;
}

const spatial::Acceleration& MassGenerator::getTargetAcceleration() const {
    return target_acceleration_.cref();
}

} // namespace phri
