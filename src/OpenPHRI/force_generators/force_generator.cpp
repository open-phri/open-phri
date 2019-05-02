/*      File: force_generator.cpp
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

#include <OpenPHRI/force_generators/force_generator.h>

using namespace phri;

ForceGenerator::ForceGenerator(ReferenceFrame frame) {
    frame_ = frame;
}

Vector6d ForceGenerator::compute() {
    Vector6d force;
    update(force);
    return transform(force);
}

Vector6d ForceGenerator::transform(const Vector6d& force) {
    if (frame_ != ReferenceFrame::TCP) {
        return robot_->spatialTransformationMatrix()->transpose() * force;
    }
    return force;
}

Vector6d ForceGenerator::operator()() {
    return compute();
}
