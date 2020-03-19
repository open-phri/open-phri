/*      File: potential_field_generator.cpp
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

#include <OpenPHRI/force_generators/potential_field_generator.h>

namespace phri {

PotentialFieldType PotentialFieldObject::type() const {
    return type_;
}
void PotentialFieldObject::setGain(const double& gain) {
    gain_.ref() = gain;
}

const double& PotentialFieldObject::getGain() const {
    return gain_.cref();
}
void PotentialFieldObject::setThresholdDistance(const double& threshold) {
    threshold_distance_.ref() = threshold;
}

const double& PotentialFieldObject::getThresholdDistance() const {
    return threshold_distance_.cref();
}
void PotentialFieldObject::setObjectPosition(
    const spatial::Position& position) {
    object_position_.ref() = position;
}

const spatial::Position& PotentialFieldObject::getObjectPosition() const {
    return object_position_.cref();
}

PotentialFieldGenerator::PotentialFieldGenerator()
    : PotentialFieldGenerator{
          spatial::LinearPosition::Zero(spatial::Frame::Unknown())} {
    std::cout << "1) this: " << this
              << ", Offset frame: " << getOffset().frame() << std::endl;
}

void PotentialFieldGenerator::update(spatial::Force& force) {
    Eigen::Vector3d total_force = Eigen::Vector3d::Zero();
    spatial::LinearPosition rob_pos{spatial::Frame::Unknown()};

    if (getOffset().frame() == spatial::Frame::Unknown()) {
        offset_.ref().changeFrame(
            spatial::Frame::Ref(robot().controlPointFrame()));
        std::cout << "2) this: " << this
                  << ", Offset frame: " << getOffset().frame() << std::endl;
    }

    for (const auto& item : items_) {
        const PotentialFieldObject& obj = item.second;
        if (obj.getObjectPosition().frame() == robot().controlPointFrame()) {
            rob_pos.changeFrame(robot().controlPointFrame());
            std::cout << "3) this: " << this
                      << ", Offset frame: " << getOffset().frame() << std::endl;
            rob_pos = getOffset();
        } else if (obj.getObjectPosition().frame() ==
                   robot().controlPointParentFrame()) {
            rob_pos.changeFrame(robot().controlPointParentFrame());
            rob_pos = robot().task().state().position().linear() +
                      robot().control().transformation() * getOffset();
        } else {
            throw std::logic_error(
                "PotentialFieldGenerator::update: a PotentialFieldObject is "
                "neither expressed in the "
                "robot control point frame or its parent");
        }
        auto obj_rob_vec = obj.getObjectPosition().linear() - rob_pos;

        double distance = obj_rob_vec.norm();
        if (std::abs(distance) > 1e-3) {
            Eigen::Vector3d obj_rob_vec_unit = obj_rob_vec.value() / distance;

            double gain = obj.getGain();

            if (obj.type() == PotentialFieldType::Attractive) {
                total_force += gain * obj_rob_vec_unit;
            } else {
                double th = obj.getThresholdDistance();
                if (distance < th) {
                    total_force +=
                        gain * (1. / th - 1. / distance) * obj_rob_vec_unit;
                }
            }
        }
    }

    force.linear() = total_force;
    force.angular().setZero();
}

void PotentialFieldGenerator::setOffset(const spatial::LinearPosition& offset) {
    offset_.ref() = offset;
}

const spatial::LinearPosition& PotentialFieldGenerator::getOffset() const {
    return offset_.cref();
}

} // namespace phri
