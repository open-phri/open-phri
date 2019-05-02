/*      File: power_constraint.cpp
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

#include <OpenPHRI/constraints/power_constraint.h>

using namespace phri;

using namespace Eigen;

/***		Constructor & destructor		***/
PowerConstraint::PowerConstraint(doubleConstPtr maximum_power)
    : maximum_power_(maximum_power) {
    power_ = std::make_shared<double>(0);
}

/***		Algorithm		***/
double PowerConstraint::compute() {
    double constraint = 1.;
    const Vector3d& velocity = robot_->control.task.total_twist.translation();
    const Vector3d& force = robot_->task.state.wrench.force();
    double power = force.dot(velocity);
    *power_ = power;

    if (power < 0.) {
        constraint = std::abs(*maximum_power_ / power);
    }

    return constraint;
}

doubleConstPtr PowerConstraint::getPower() const {
    return power_;
}
