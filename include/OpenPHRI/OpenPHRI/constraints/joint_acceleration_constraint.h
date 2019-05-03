/*      File: joint_acceleration_constraint.h
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
 * @file joint_acceleration_constraint.h
 * @author Benjamin Navarro
 * @brief Definition of the JointAccelerationConstraint class
 * @date October 2018
 * @ingroup phri
 */

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/constraints/constraint.h>

namespace phri {

/** @brief A constraint to limit the joint acceleration.
 */
class JointAccelerationConstraint : public Constraint {
public:
    /***		Constructor & destructor		***/

    /**
     * @brief Construct an acceleration constraint with a given maximum
     * acceleration.
     * @param maximum_acceleration A shared pointer to the maximum acceleration
     * allowed.
     */
    JointAccelerationConstraint(VectorXdConstPtr maximum_acceleration,
                                double sample_time);

    virtual ~JointAccelerationConstraint() = default;

    /***		Algorithm		***/
    virtual double compute() override;

private:
    VectorXdConstPtr maximum_acceleration_;
    double sample_time_;
};

} // namespace phri
