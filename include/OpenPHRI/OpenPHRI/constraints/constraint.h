/*      File: constraint.h
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
 * @file constraint.h
 * @author Benjamin Navarro
 * @brief Base classes definitions for constraint implementation
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/robot.h>
#include <OpenPHRI/fwd_decl.h>

namespace phri {

/** @brief Base class for all constraints.
 *  @details Provides a ConstraintType and a pure virtual compute method.
 */
class Constraint {
public:
    /**
     * @brief Construct a constraint of a given type
     * @param type The type of the constraint. See ConstraintType.
     */
    Constraint() = default;
    virtual ~Constraint() = default;

    /**
     * @brief Compute the value associated with the constraint.
     * @return The constraint's evaluated value.
     */
    virtual double compute() = 0;

    /**
     * @brief Call operator, shortcut for compute.
     * @return The constraint's evaluated value.
     */
    virtual double operator()() final;

protected:
    friend class SafetyController;
    friend class SeparationDistanceConstraint;

    /**
     * @brief Set the robot to work with.
     * @param robot The robot.
     */
    virtual void setRobot(Robot const* robot);

    Robot const* robot_;
};

using ConstraintPtr = std::shared_ptr<Constraint>;
using ConstraintConstPtr = std::shared_ptr<const Constraint>;

} // namespace phri
