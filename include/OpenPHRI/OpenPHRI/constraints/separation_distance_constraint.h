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

// TODO check and possibly improve the separation distance constraint

namespace phri {

/** @brief A meta-constraint to adapt a given constraint depending on the
 * distance to the closest object.
 *  @details You have to provide preconfigured constraint and interpolator.
 *  The interpolator input is set to the serapration distance.
 */
class SeparationDistanceConstraint
    : public Constraint,
      public ObjectCollection<spatial::Position> {
public:
    /**
     * @brief Construct a separaration distance constraint with a given
     * constraint and interpolator. Objects position must be expressed in the
     * TCP frame.
     * @param constraint The constraint to wrap.
     * @param interpolator The interpolator used to tune the constraint.
     */
    SeparationDistanceConstraint(std::shared_ptr<Constraint> constraint,
                                 std::shared_ptr<Interpolator> interpolator);

    /**
     * @brief Construct a separaration distance constraint with a given
     * constraint, interpolator and robot positon. Objects position must be
     * expressed in the same frame as robot_position.
     * @param constraint The constraint to wrap.
     * @param interpolator The interpolator used to tune the constraint.
     * @param robot_position The positon of the robot in the same frame as the
     * objects.
     */
    SeparationDistanceConstraint(
        std::shared_ptr<Constraint> constraint,
        std::shared_ptr<Interpolator> interpolator,
        std::shared_ptr<const spatial::Position> robot_position);

    double compute() override;

    /**
     * @brief Retrieve the separation shared pointer.
     * @return The shared pointer to the separation power.
     */
    std::shared_ptr<const double> getSeparationDistance() const;

protected:
    void setRobot(Robot const* robot) override;

private:
    double closestObjectDistance();

    std::shared_ptr<Constraint> constraint_;
    std::shared_ptr<Interpolator> interpolator_;
    std::shared_ptr<const spatial::Position> robot_position_;
    std::shared_ptr<double> separation_distance_;
};

} // namespace phri
