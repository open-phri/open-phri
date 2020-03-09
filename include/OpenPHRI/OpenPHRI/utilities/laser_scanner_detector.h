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
    LaserScannerDetector(std::shared_ptr<const Eigen::VectorXd> laser_data,
                         double scanning_angle, double minimum_distance,
                         double maximum_distance, double threshold = 0.1);

    ~LaserScannerDetector() = default;

    std::shared_ptr<const Eigen::Vector2d> getPosition() const;
    std::shared_ptr<const double> getDistance() const;

    void init();
    double compute();
    double operator()();

private:
    std::shared_ptr<Eigen::Vector2d> position_;
    std::shared_ptr<double> distance_;
    std::shared_ptr<const Eigen::VectorXd> laser_data_;
    double scanning_angle_;
    double minimum_distance_;
    double maximum_distance_;
    double threshold_;

    Eigen::VectorXd maximum_distances_;
};

} // namespace phri
