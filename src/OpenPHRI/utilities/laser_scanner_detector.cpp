/*      File: laser_scanner_detector.cpp
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

#include <OpenPHRI/utilities/laser_scanner_detector.h>
#include <iostream>

using namespace phri;

LaserScannerDetector::LaserScannerDetector(
    std::shared_ptr<const Eigen::VectorXd> laser_data, double scanning_angle,
    double minimum_distance, double maximum_distance, double threshold)
    : laser_data_(laser_data),
      scanning_angle_(scanning_angle),
      minimum_distance_(minimum_distance),
      maximum_distance_(maximum_distance),
      threshold_(threshold) {
    position_ = std::make_shared<Eigen::Vector2d>();
    distance_ = std::make_shared<double>();
}

std::shared_ptr<const Eigen::Vector2d>
LaserScannerDetector::getPosition() const {
    return position_;
}

std::shared_ptr<const double> LaserScannerDetector::getDistance() const {
    return distance_;
}

void LaserScannerDetector::init() {
    maximum_distances_.resize(laser_data_->size());
    const auto& dist = *laser_data_;
    for (size_t i = 0; i < laser_data_->size(); ++i) {
        maximum_distances_(i) =
            dist(i) < maximum_distance_ && dist(i) > minimum_distance_
                ? dist(i)
                : maximum_distance_;
    }
}

double LaserScannerDetector::compute() {
    double step_size = scanning_angle_ / laser_data_->size();
    int min_dist_idx = -1;
    const auto& dist = *laser_data_;
    double min_dist = maximum_distance_;

    for (size_t i = 0; i < dist.size(); ++i) {
        double di = dist(i);
        if ((di < maximum_distances_(i) - threshold_) &&
            (di >= minimum_distance_) && (di < min_dist)) {
            min_dist_idx = i;
            min_dist = di;
        }
    }

    if (min_dist_idx >= 0) {
        double angle = (min_dist_idx - laser_data_->size() / 2) * step_size;
        position_->x() = min_dist * std::cos(angle);
        position_->y() = min_dist * std::sin(angle);
    } else {
        position_->x() = maximum_distance_;
        position_->y() = 0.;
    }

    // std::cout << "position: " << position_->transpose() << ", min_dist: " <<
    // min_dist << std::endl;

    *distance_ = min_dist;
    return min_dist;
}

double LaserScannerDetector::operator()() {
    return compute();
}
