/*      File: laser_scanner_detector.h
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
 * @file laser_scanner_detector.h
 * @author Benjamin Navarro
 * @brief Definition of the LaserScannerDetector class.
 * @date June 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/definitions.h>

namespace phri {

class LaserScannerDetector {
public:
    LaserScannerDetector(VectorXdConstPtr laser_data, double scanning_angle,
                         double minimum_distance, double maximum_distance,
                         double threshold = 0.1);

    ~LaserScannerDetector() = default;

    Vector2dConstPtr getPosition() const;
    doubleConstPtr getDistance() const;

    void init();
    double compute();
    double operator()();

private:
    Vector2dPtr position_;
    doublePtr distance_;
    VectorXdConstPtr laser_data_;
    double scanning_angle_;
    double minimum_distance_;
    double maximum_distance_;
    double threshold_;

    VectorXd maximum_distances_;
};

} // namespace phri
