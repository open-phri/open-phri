/*      File: stop_constraint.h
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
 * @file stop_constraint.h
 * @author Benjamin Navarro
 * @brief Definition of the EmergencyStopConstraint class
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/constraints/constraint.h>

namespace phri {

/** @brief A constraint to stop the robot when a contant occurs.
 *  @details Two thresholds are used. The robot is stopped when the external
 * force is above the activation threshold and released when the external force
 * gets below the deactivation threshold.
 */
class EmergencyStopConstraint : public Constraint {
public:
    enum CheckType {
        CheckForces = 1 << 0,
        CheckJointTorques = 1 << 1,
        CheckBoth = CheckForces | CheckJointTorques
    };

    /***		Constructor & destructor		***/

    /**
     * @brief Construct a stop constraint with a given external force,
     * activation and deactivation threshold.
     * @param activation_force_threshold A shared pointer to the activation
     * threshold.
     * @param deactivation_force_threshold A shared pointer to the deactivation
     * threshold.
     */
    EmergencyStopConstraint(doubleConstPtr activation_force_threshold,
                            doubleConstPtr deactivation_force_threshold);

    EmergencyStopConstraint(CheckType check_type,
                            doubleConstPtr activation_force_threshold,
                            doubleConstPtr deactivation_force_threshold,
                            doubleConstPtr activation_torque_threshold,
                            doubleConstPtr deactivation_torque_threshold);

    virtual ~EmergencyStopConstraint() = default;

    /***		Algorithm		***/
    virtual double compute() override;

private:
    doubleConstPtr activation_force_threshold_;
    doubleConstPtr deactivation_force_threshold_;
    doubleConstPtr activation_torque_threshold_;
    doubleConstPtr deactivation_torque_threshold_;

    CheckType check_type_;
    double previous_constraint_value_;
};

using EmergencyStopConstraintPtr = std::shared_ptr<EmergencyStopConstraint>;
using EmergencyStopConstraintConstPtr =
    std::shared_ptr<const EmergencyStopConstraint>;

} // namespace phri
