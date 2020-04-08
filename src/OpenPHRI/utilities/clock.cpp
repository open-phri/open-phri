/*      File: clock.cpp
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

#include <OpenPHRI/utilities/clock.h>

namespace phri {

Clock::Clock() : sample_time_(-1.) {
    reset();
    time_ = std::make_shared<double>(0.);
}

Clock::Clock(double sample_time)
    : sample_time_(sample_time), time_(std::make_shared<double>(0.)) {
}

void Clock::reset() {
    if (sample_time_ < 0) {
        init_time_ = std::chrono::high_resolution_clock::now();
    } else {
        *time_ = 0.;
    }
}

std::shared_ptr<double> Clock::getTime() const {
    return time_;
}

double Clock::update() {
    auto& current_time = *time_;
    if (sample_time_ <= 0) {
        current_time =
            std::chrono::duration<double>(
                std::chrono::high_resolution_clock::now() - init_time_)
                .count();
    } else {
        current_time += sample_time_;
    }
    return current_time;
}

double Clock::operator()() {
    return update();
}

} // namespace phri
