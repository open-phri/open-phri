/*
 *  Copyright (C) 2017 Benjamin Navarro <contact@bnavarro.info>
 *
 *  This file is part of OpenPHRI <https://gite.lirmm.fr/navarro/OpenPHRI>.
 *
 *  OpenPHRI is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  OpenPHRI is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with OpenPHRI.  If not, see <http://www.gnu.org/licenses/>.
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
	LaserScannerDetector(
		VectorXdConstPtr laser_data,
		double scanning_angle,
		double minimum_distance,
		double maximum_distance,
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
