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

namespace phri {

// ForceGenerator::ForceGenerator(spatial::Frame frame) : wrench_{frame} {
// }

const spatial::Force& ForceGenerator::compute() {
    wrench_.setZero();
    update(wrench_);
    return wrench_;
}

const spatial::Force& ForceGenerator::operator()() {
    return compute();
}

const spatial::Frame& ForceGenerator::frame() const {
    return internalWrench().frame();
}

void ForceGenerator::setRobot(Robot const* robot) {
    robot_ = robot;
}

const Robot& ForceGenerator::robot() const {
    return *robot_;
}

void ForceGenerator::setFrame(const spatial::Frame& frame) {
    wrench_.changeFrame(frame);
}

const spatial::Force& ForceGenerator::internalWrench() const {
    return wrench_;
}

} // namespace phri