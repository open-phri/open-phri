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

StiffnessGenerator::StiffnessGenerator(ReferenceFrame stiffness_frame)
    : StiffnessGenerator(std::make_shared<Vector6d>(Vector6d::Zero()),
                         std::make_shared<Pose>(), stiffness_frame) {
}

StiffnessGenerator::StiffnessGenerator(std::shared_ptr<Vector6d> stiffness,
                                       std::shared_ptr<Pose> target_pose,
                                       ReferenceFrame stiffness_frame)
    : stiffness_(stiffness),
      target_pose_(target_pose),
      stiffness_frame_(stiffness_frame) {
    if (not stiffness or not target_pose) {
        throw std::runtime_error(
            OPEN_PHRI_ERROR("You provided an empty shared pointer"));
    }
}

StiffnessGenerator::StiffnessGenerator(Vector6d& stiffness, Pose& target_pose,
                                       ReferenceFrame stiffness_frame)
    : StiffnessGenerator(std::shared_ptr<Vector6d>(&stiffness, [](auto p) {}),
                         std::shared_ptr<Pose>(&target_pose, [](auto p) {}),
                         stiffness_frame) {
}

StiffnessGenerator::StiffnessGenerator(const Vector6d& stiffness,
                                       const Pose& target_pose,
                                       ReferenceFrame stiffness_frame)
    : StiffnessGenerator(std::make_shared<Vector6d>(stiffness),
                         std::make_shared<Pose>(target_pose), stiffness_frame) {
}

StiffnessGenerator::StiffnessGenerator(Vector6d&& stiffness, Pose&& target_pose,
                                       ReferenceFrame stiffness_frame)
    : StiffnessGenerator(std::make_shared<Vector6d>(std::move(stiffness)),
                         std::make_shared<Pose>(std::move(target_pose)),
                         stiffness_frame) {
}

void StiffnessGenerator::update(Wrench& force) {
    // TODO rewrite
    // Vector6d error;

    // if (stiffness_frame_ == ReferenceFrame::TCP) {
    //     error = robot().control.spatial_transformation_matrix.transpose() *
    //             (robot().task.state.pose.getErrorWith(*target_pose_));
    // } else {
    //     error = robot().task.state.pose.getErrorWith(*target_pose_);
    // }

    // error.segment<3>(0) *= -1.;
    // force = stiffness_->cwiseProduct(error);
}

Vector6d& StiffnessGenerator::stiffness() {
    return *stiffness_;
}

const Vector6d& StiffnessGenerator::stiffness() const {
    return *stiffness_;
}

std::shared_ptr<Vector6d> StiffnessGenerator::stiffnessPtr() const {
    return stiffness_;
}

Pose& StiffnessGenerator::targetPose() {
    return *target_pose_;
}

const Pose& StiffnessGenerator::targetPose() const {
    return *target_pose_;
}

std::shared_ptr<Pose> StiffnessGenerator::targetPosePtr() const {
    return target_pose_;
}

} // namespace phri
