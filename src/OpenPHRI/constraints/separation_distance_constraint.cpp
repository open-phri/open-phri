/*      File: separation_distance_constraint.cpp
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

#include <OpenPHRI/constraints/separation_distance_constraint.h>

using namespace phri;

SeparationDistanceConstraint::SeparationDistanceConstraint(
    std::shared_ptr<Constraint> constraint, InterpolatorPtr interpolator) {
    constraint_ = constraint;
    interpolator_ = interpolator;

    separation_distance_ = std::make_shared<double>();

    interpolator->setInput(separation_distance_);

    robot_position_ = std::make_shared<Vector6d>(Vector6d::Zero());
}

SeparationDistanceConstraint::SeparationDistanceConstraint(
    std::shared_ptr<Constraint> constraint, InterpolatorPtr interpolator,
    std::shared_ptr<const Vector6d> robot_position)
    : SeparationDistanceConstraint(constraint, interpolator) {
    robot_position_ = robot_position;
}

double SeparationDistanceConstraint::compute() {
    *separation_distance_ = closestObjectDistance();
    interpolator_->compute();
    return constraint_->compute();
}

std::shared_ptr<const double>
SeparationDistanceConstraint::getSeparationDistance() const {
    return separation_distance_;
}

void SeparationDistanceConstraint::setRobot(Robot const* robot) {
    constraint_->setRobot(robot);
    Constraint::setRobot(robot);
}

double SeparationDistanceConstraint::closestObjectDistance() {
    const Vector3d& rob_pos = robot_position_->block<3, 1>(0, 0);

    double min_dist = std::numeric_limits<double>::infinity();
    for (const auto& item : items_) {
        Vector3d obj_rob_vec = item.second->translation() - rob_pos;

        min_dist = std::min(min_dist, obj_rob_vec.norm());
    }

    return min_dist;
}
