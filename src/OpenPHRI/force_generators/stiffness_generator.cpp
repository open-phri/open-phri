/*      File: stiffness_generator.cpp
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

#include <OpenPHRI/force_generators/stiffness_generator.h>
#include <OpenPHRI/utilities/exceptions.h>

#include <physical_quantities/spatial/impedance/stiffness.hpp>

namespace phri {

StiffnessGenerator::StiffnessGenerator()
    : stiffness_(spatial::Stiffness::Zero(spatial::Frame::Ref(frame()))),
      target_pose_(spatial::Position::Zero(spatial::Frame::Ref(frame()))) {
}

void StiffnessGenerator::update(spatial::Force& force) {
    auto to_cp_frame = [this](const auto& value) {
        if (value.frame() != robot().controlPointFrame()) {
            return robot().control().transformation().inverse() * value;
        } else {
            return value;
        }
    };

    auto current_pose_cp = to_cp_frame(robot().task().state().position());
    auto target_pose_cp = to_cp_frame(getTargetPose());
    auto stiffness_cp = to_cp_frame(getStiffness());

    force = stiffness_cp * (target_pose_cp - current_pose_cp);
}

void StiffnessGenerator::setStiffness(const spatial::Stiffness& stiffness) {
    stiffness_.ref() = stiffness;
}

const spatial::Stiffness& StiffnessGenerator::getStiffness() const {
    return stiffness_.cref();
}
void StiffnessGenerator::setTargetPose(const spatial::Position& position) {
    target_pose_.ref() = position;
}

const spatial::Position& StiffnessGenerator::getTargetPose() const {
    return target_pose_.cref();
}

} // namespace phri
