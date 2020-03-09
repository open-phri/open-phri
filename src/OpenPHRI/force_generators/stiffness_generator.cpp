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

namespace phri {

StiffnessGenerator::StiffnessGenerator(
    std::shared_ptr<spatial::Stiffness> stiffness,
    std::shared_ptr<spatial::Position> target_pose)
    : stiffness_(stiffness), target_pose_(target_pose) {
    if (not stiffness or not target_pose) {
        throw std::runtime_error(
            OPEN_PHRI_ERROR("You provided an empty shared pointer"));
    }
}

StiffnessGenerator::StiffnessGenerator(spatial::Stiffness& stiffness,
                                       spatial::Position& target_pose)
    : StiffnessGenerator(
          std::shared_ptr<spatial::Stiffness>(&stiffness, [](auto p) {}),
          std::shared_ptr<spatial::Position>(&target_pose, [](auto p) {})) {
}

StiffnessGenerator::StiffnessGenerator(const spatial::Stiffness& stiffness,
                                       const spatial::Position& target_pose)
    : StiffnessGenerator(std::make_shared<spatial::Stiffness>(stiffness),
                         std::make_shared<spatial::Position>(target_pose)) {
}

StiffnessGenerator::StiffnessGenerator(spatial::Stiffness&& stiffness,
                                       spatial::Position&& target_pose)
    : StiffnessGenerator(
          std::make_shared<spatial::Stiffness>(std::move(stiffness)),
          std::make_shared<spatial::Position>(std::move(target_pose))) {
}

void StiffnessGenerator::update(spatial::Force& force) {
    // TODO rewrite
    // spatial::Stiffness error;

    // if (stiffness_frame_ == spatial::Frame::TCP) {
    //     error = robot().control().spatial_transformation_matrix.transpose() *
    //             (robot().task().state.pose.getErrorWith(*target_pose_));
    // } else {
    //     error = robot().task().state.pose.getErrorWith(*target_pose_);
    // }

    // error.segment<3>(0) *= -1.;
    // force = stiffness_->cwiseProduct(error);
}

spatial::Stiffness& StiffnessGenerator::stiffness() {
    return *stiffness_;
}

const spatial::Stiffness& StiffnessGenerator::stiffness() const {
    return *stiffness_;
}

std::shared_ptr<spatial::Stiffness> StiffnessGenerator::stiffnessPtr() const {
    return stiffness_;
}

spatial::Position& StiffnessGenerator::targetPose() {
    return *target_pose_;
}

const spatial::Position& StiffnessGenerator::targetPose() const {
    return *target_pose_;
}

std::shared_ptr<spatial::Position> StiffnessGenerator::targetPosePtr() const {
    return target_pose_;
}

} // namespace phri
