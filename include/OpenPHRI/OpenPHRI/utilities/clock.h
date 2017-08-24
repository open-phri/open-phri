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
	Clock(double sample_time);
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
