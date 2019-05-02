/*      File: velocity_constraint.h
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
 * @file velocity_constraint.h
 * @author Benjamin Navarro
 * @brief Definition of the VelocityConstraint class
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/constraints/constraint.h>

namespace phri {

/** @brief A constraint to limit the TCP velocity.
 */
class VelocityConstraint : public Constraint {
public:
    /***		Constructor & destructor		***/

    /**
     * @brief Construct a velocity constraint with a given total and a maximum
     * velocity.
     * @param maximum_velocity A shared pointer to the maximum velocity allowed.
     */
    explicit VelocityConstraint(doubleConstPtr maximum_velocity);

    virtual ~VelocityConstraint() = default;

    /***		Algorithm		***/
    virtual double compute() override;

protected:
    VelocityConstraint() = default;
    doubleConstPtr maximum_velocity_;
};

using VelocityConstraintPtr = std::shared_ptr<VelocityConstraint>;
using VelocityConstraintConstPtr = std::shared_ptr<const VelocityConstraint>;

} // namespace phri
