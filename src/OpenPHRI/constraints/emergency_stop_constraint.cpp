/*      File: stop_constraint.cpp
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

#include <OpenPHRI/constraints/emergency_stop_constraint.h>
#include <iostream>

using namespace phri;

using namespace Eigen;

/***		Constructor & destructor		***/
EmergencyStopConstraint::EmergencyStopConstraint(
    doubleConstPtr activation_force_threshold,
    doubleConstPtr deactivation_force_threshold)
    : activation_force_threshold_(activation_force_threshold),
      deactivation_force_threshold_(deactivation_force_threshold),
      check_type_(CheckType::CheckForces),
      previous_constraint_value_(1.) {
}

EmergencyStopConstraint::EmergencyStopConstraint(
    CheckType check_type, doubleConstPtr activation_force_threshold,
    doubleConstPtr deactivation_force_threshold,
    doubleConstPtr activation_torque_threshold,
    doubleConstPtr deactivation_torque_threshold)
    : activation_force_threshold_(activation_force_threshold),
      deactivation_force_threshold_(deactivation_force_threshold),
      activation_torque_threshold_(activation_torque_threshold),
      deactivation_torque_threshold_(deactivation_torque_threshold),
      check_type_(check_type),
      previous_constraint_value_(1.) {
}

/***		Algorithm		***/
double EmergencyStopConstraint::compute() {
    double constraint;
    double force_constraint = 1.;
    double torque_constraint = 1.;

    if (check_type_ & CheckType::CheckForces) {
        double act = *activation_force_threshold_;
        double deact = *deactivation_force_threshold_;
        double norm =
            robot_->controlPointExternalForce()->block<3, 1>(0, 0).norm();

        if (norm >= act) {
            force_constraint = 0.;
        } else if (norm <= deact) {
            force_constraint = 1.;
        } else {
            force_constraint = previous_constraint_value_;
        }
    }

    if (check_type_ & CheckType::CheckJointTorques) {
        double act = *activation_torque_threshold_;
        double deact = *deactivation_torque_threshold_;
        double norm = robot_->jointExternalTorque()->norm();

        if (norm >= act) {
            torque_constraint = 0.;
        } else if (norm <= deact) {
            torque_constraint = 1.;
        } else {
            torque_constraint = previous_constraint_value_;
        }
    }

    // Should be > 1 but we never know with floating point numbers
    constraint = force_constraint + torque_constraint > 1.5 ? 1. : 0.;
    previous_constraint_value_ = constraint;

    return constraint;
}
