/*      File: velocity_generator.cpp
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

#include <OpenPHRI/velocity_generators/velocity_generator.h>

using namespace phri;

VelocityGenerator::VelocityGenerator(ReferenceFrame frame) {
    frame_ = frame;
}

Twist VelocityGenerator::compute() {
    Twist velocity;
    update(velocity);
    return transform(velocity);
}

Twist VelocityGenerator::transform(const Twist& velocity) {
    if (frame_ != ReferenceFrame::TCP) {
        return static_cast<Twist>(
            robot_->control.spatial_transformation_matrix.transpose() *
            static_cast<Vector6d>(velocity));
    }
    return velocity;
}

Twist VelocityGenerator::operator()() {
    return compute();
}

void VelocityGenerator::setRobot(Robot const* robot) {
    robot_ = robot;
}