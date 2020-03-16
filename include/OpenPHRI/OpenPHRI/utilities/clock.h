/*      File: clock.h
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

/**
 * @file clock.h
 * @author Benjamin Navarro
 * @brief Definition of the Clock class
 * @date June 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <memory>
#include <chrono>

namespace phri {

class Clock {
public:
    Clock();
    explicit Clock(double sample_time);
    ~Clock() = default;

    void reset();
    std::shared_ptr<double> getTime() const;

    double update();
    double operator()();

private:
    std::chrono::high_resolution_clock::time_point init_time_;
    double sample_time_;
    std::shared_ptr<double> time_;
};

} // namespace phri
