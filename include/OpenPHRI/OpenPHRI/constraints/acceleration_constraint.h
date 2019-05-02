/*      File: acceleration_constraint.h
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
 * @file acceleration_constraint.h
 * @author Benjamin Navarro
 * @brief Definition of the AccelerationConstraint class
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/constraints/constraint.h>

namespace phri {

/** @brief A constraint to limit the TCP acceleration.
 */
class AccelerationConstraint : public Constraint {
public:
    /***		Constructor & destructor		***/

    /**
     * @brief Construct a acceleration constraint with a given total and a
     * maximum acceleration.
     * @param maximum_acceleration A shared pointer to the maximum acceleration
     * allowed.
     */
    AccelerationConstraint(doubleConstPtr maximum_acceleration,
                           double sample_time);

    virtual ~AccelerationConstraint() = default;

    /***		Algorithm		***/
    virtual double compute() override;

private:
    doubleConstPtr maximum_acceleration_;
    double sample_time_;
};

using AccelerationConstraintPtr = std::shared_ptr<AccelerationConstraint>;
using AccelerationConstraintConstPtr =
    std::shared_ptr<const AccelerationConstraint>;

} // namespace phri
