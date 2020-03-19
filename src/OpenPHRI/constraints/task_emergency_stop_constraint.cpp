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

    if (norm >= getActivationThreshold()) {
        constraint = 0.;
    } else if (norm <= getDeactivationThreshold()) {
        constraint = 1.;
    } else {
        constraint = previous_constraint_value_;
    }

    previous_constraint_value_ = constraint;

    return constraint;
}

void TaskEmergencyStopConstraint::setActivationThreshold(
    const scalar::Force& threshold) {
    activation_threshold_.ref() = threshold;
}

const scalar::Force&
TaskEmergencyStopConstraint::getActivationThreshold() const {
    return activation_threshold_.cref();
}

void TaskEmergencyStopConstraint::setDeactivationThreshold(
    const scalar::Force& threshold) {
    deactivation_threshold_.ref() = threshold;
}

const scalar::Force&
TaskEmergencyStopConstraint::getDeactivationThreshold() const {
    return deactivation_threshold_.cref();
}

} // namespace phri