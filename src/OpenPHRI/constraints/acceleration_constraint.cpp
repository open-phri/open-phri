/*      File: acceleration_constraint.cpp
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

#include <OpenPHRI/constraints/acceleration_constraint.h>

using namespace phri;

using namespace Eigen;

/***		Constructor & destructor		***/
AccelerationConstraint::AccelerationConstraint(
    doubleConstPtr maximum_acceleration, double sample_time)
    : maximum_acceleration_(maximum_acceleration), sample_time_(sample_time) {
}

/***		Algorithm		***/
double AccelerationConstraint::compute() {
    double constraint = 1.;
    double v_norm = robot_->controlPointTotalVelocity()->translation().norm();

    if (v_norm > 0.) {
        double prev_v_norm =
            robot_->controlPointVelocity()->translation().norm();
        double vmax =
            prev_v_norm + std::abs(*maximum_acceleration_) * sample_time_;
        constraint = vmax / v_norm;
    }

    return constraint;
}
