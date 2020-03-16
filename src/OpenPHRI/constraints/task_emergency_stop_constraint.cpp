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

#include <OpenPHRI/constraints/task_emergency_stop_constraint.h>
#include <OpenPHRI/utilities/exceptions.h>

namespace phri {

TaskEmergencyStopConstraint::TaskEmergencyStopConstraint()
    : activation_threshold_(scalar::Force{0.}),
      deactivation_threshold_(scalar::Force{0.}),
      previous_constraint_value_(1.) {
}

double TaskEmergencyStopConstraint::compute() {
    double constraint;

    scalar::Force norm{robot_->task().state().force().linear().norm()};

    if (norm >= activationThreshold()) {
        constraint = 0.;
    } else if (norm <= deactivationThreshold()) {
        constraint = 1.;
    } else {
        constraint = previous_constraint_value_;
    }

    previous_constraint_value_ = constraint;

    return constraint;
}

scalar::Force& TaskEmergencyStopConstraint::activationThreshold() {
    return activation_threshold_;
}

const scalar::Force& TaskEmergencyStopConstraint::activationThreshold() const {
    return activation_threshold_;
}

scalar::Force& TaskEmergencyStopConstraint::deactivationThreshold() {
    return deactivation_threshold_;
}

const scalar::Force&
TaskEmergencyStopConstraint::deactivationThreshold() const {
    return deactivation_threshold_;
}

} // namespace phri