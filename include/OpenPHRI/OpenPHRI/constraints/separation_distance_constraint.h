/*      File: separation_distance_constraint.h
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

/**
 * @file separation_distance_constraint.h
 * @author Benjamin Navarro
 * @brief Definition of the SeparationDistanceConstraint class
 * @date April 2017
 * @ingroup phri
 */

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/constraints/constraint.h>
#include <OpenPHRI/utilities/interpolator.h>
#include <OpenPHRI/utilities/object_collection.hpp>
#include <map>

#include <physical_quantities/spatial/position.h>

namespace phri {

/** @brief A meta-constraint to adapt a given constraint depending on the
 * distance to the closest object.
 *  @details You have to provide preconfigured constraint and interpolator.
 *  The interpolator input is set to the serapration distance.
 */
template <typename ConstraintT, typename InterpolatorT>
class SeparationDistanceConstraint
    : public Constraint,
      public ObjectCollection<spatial::Position> {
public:
    SeparationDistanceConstraint() : constraint_{}, interpolator_{} {
        interpolator().setInput(&separation_distance_);
    }

    SeparationDistanceConstraint(ConstraintT&& constraint,
                                 InterpolatorT&& interpolator)
        : constraint_{std::move(constraint)},
          interpolator_{std::move(interpolator)} {
        this->interpolator().setInput(&separation_distance_);
    }

    SeparationDistanceConstraint(
        ConstraintT&& constraint, InterpolatorT&& interpolator,
        std::shared_ptr<const spatial::Position> robot_position)
        : SeparationDistanceConstraint(std::forward(constraint),
                                       std::forward(interpolator)) {
        robot_position_ = robot_position;
    }

    virtual ~SeparationDistanceConstraint() = default;

    [[nodiscard]] double compute() override {
        separation_distance_ = closestObjectDistance();
        interpolator().compute();
        return constraint().compute();
    }

    [[nodiscard]] const scalar::Position& separationDistance() const {
        return separation_distance_;
    }

    [[nodiscard]] ConstraintT& constraint() {
        return constraint_;
    }

    [[nodiscard]] const ConstraintT& constraint() const {
        return constraint_;
    }

    [[nodiscard]] InterpolatorT& interpolator() {
        return interpolator_;
    }

    [[nodiscard]] const InterpolatorT& interpolator() const {
        return interpolator_;
    }

    void setRobot(Robot const* robot) override {
        constraint().setRobot(robot);
        Constraint::setRobot(robot);
        if (not robot_position_) {
            robot_position_ = std::make_shared<spatial::Position>(
                spatial::Position::Zero(robot->controlPointParentFrame()));
        }
    }

    [[nodiscard]] const spatial::Position& robotPosition() const {
        return *robot_position_;
    }

private:
    [[nodiscard]] scalar::Position closestObjectDistance() {
        Eigen::Vector3d rob_pos = Eigen::Vector3d::Zero();
        if (robotPosition().frame()) {
            rob_pos = robotPosition().linear();
        }

        auto min_dist =
            scalar::Position{std::numeric_limits<double>::infinity()};
        for (const auto& item : items_) {
            const auto& obj = item.second.cref();
            Eigen::Vector3d obj_rob_vec = obj.linear() - rob_pos;

            min_dist.value() = std::min(min_dist.value(), obj_rob_vec.norm());
        }

        return min_dist;
    }

    ConstraintT constraint_;
    InterpolatorT interpolator_;
    std::shared_ptr<const spatial::Position> robot_position_;
    scalar::Position separation_distance_;
};

} // namespace phri
