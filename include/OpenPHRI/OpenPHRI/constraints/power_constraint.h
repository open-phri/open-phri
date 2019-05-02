/*      File: power_constraint.h
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
 * @file power_constraint.h
 * @author Benjamin Navarro
 * @brief Definition of the PowerConstraint class
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/constraints/constraint.h>

namespace phri {

/** @brief A constraint to limit the exchanged power.
 *  @details The power considered is the dot product of the total velocity with
 * the external force. The constraint is active only when the power is negative,
 * which is when the robot pushes the environment. No limitation is applied when
 * the robot is pushed by the environment.
 */
class PowerConstraint : public Constraint {
public:
    /***		Constructor & destructor		***/

    /**
     * @brief Construct a power constraint with a given total velocity, external
     * force and maximum power.
     * @param maximum_power A shared pointer to the maximum power allowed.
     */
    explicit PowerConstraint(doubleConstPtr maximum_power);

    virtual ~PowerConstraint() = default;

    /***		Algorithm		***/
    virtual double compute() override;

    /**
     * @brief Retrieve the computed power shared pointer.
     * @return The shared pointer to the computed power.
     */
    doubleConstPtr getPower() const;

private:
    Vector6dConstPtr external_force_;
    doubleConstPtr maximum_power_;

    doublePtr power_;
};

using PowerConstraintPtr = std::shared_ptr<PowerConstraint>;
using PowerConstraintConstPtr = std::shared_ptr<const PowerConstraint>;

} // namespace phri
