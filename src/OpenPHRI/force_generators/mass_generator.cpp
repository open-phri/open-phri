/*      File: mass_generator.cpp
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

#include <OpenPHRI/force_generators/mass_generator.h>

using namespace phri;

MassGenerator::MassGenerator(Matrix6dConstPtr mass,
                             AccelerationConstPtr target_acceleration,
                             ReferenceFrame mass_frame,
                             ReferenceFrame target_acceleration_frame)
    : ForceGenerator(mass_frame),
      mass_(mass),
      target_acceleration_(target_acceleration),
      mass_frame_(mass_frame),
      target_acceleration_frame_(target_acceleration_frame) {
}

void MassGenerator::update(Vector6d& force) {
    Vector6d error;

    if (mass_frame_ == ReferenceFrame::TCP) {
        if (target_acceleration_frame_ == ReferenceFrame::TCP) {
            error = *target_acceleration_;
        } else {
            error = robot_->spatialTransformationMatrix()->transpose() *
                    (static_cast<const Vector6d&>(*target_acceleration_) -
                     static_cast<const Vector6d&>(
                         *robot_->controlPointCurrentAcceleration()));
        }
    } else {
        if (target_acceleration_frame_ == ReferenceFrame::TCP) {
            error = *robot_->spatialTransformationMatrix() *
                    static_cast<const Vector6d&>(*target_acceleration_);
        } else {
            error = static_cast<const Vector6d&>(*target_acceleration_) -
                    static_cast<const Vector6d&>(
                        *robot_->controlPointCurrentAcceleration());
        }
    }

    force = *mass_ * error;
}
