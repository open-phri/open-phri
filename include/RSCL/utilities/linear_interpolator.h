/*
 *  Copyright (C) 2017 Benjamin Navarro <contact@bnavarro.info>
 *
 *  This file is part of RSCL <https://gite.lirmm.fr/navarro/RSCL>.
 *
 *  RSCL is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  RSCL is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with RSCL.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file linear_interpolator.h
 * @author Benjamin Navarro
 * @brief Definition of the LinearInterpolator class
 * @date April 2017
 * @ingroup RSCL
 */

#pragma once

#include <RSCL/utilities/interpolator.h>
#include <RSCL/utilities/interpolators_common.h>

namespace RSCL {

/** @brief A linear interpolator with an optional saturation.
 */
class LinearInterpolator : public Interpolator {
public:
	/**
	 * @brief Construct a linear interpolator given starting and ending points and a user defined input
	 * @param from Starting point.
	 * @param to Ending point.
	 * @param input A shared pointer to the input value used by the interpolator.
	 */
	LinearInterpolator(LinearPointConstPtr from, LinearPointConstPtr to, doubleConstPtr input);

	/**
	 * @brief Construct a linear interpolator given starting and ending points.
	 * The input pointer can be set with Interpolator::setInput. Maily useful when embedded in an other class.
	 * @param from Starting point.
	 * @param to Ending point.
	 */
	LinearInterpolator(LinearPointConstPtr from, LinearPointConstPtr to);

	~LinearInterpolator() = default;

	/**
	 * @brief Turn the saturation on or off. If on, the output will be restricted to the [from.y() to.y()] range.
	 * @param on True to enable the saturation, false otherwise.
	 */
	void enableSaturation(bool on);

	virtual void computeParameters() override;

	virtual double compute() override;

private:
	struct LinearParameters {
		double a;
		double b;

		double xi;
		double xf;
		double yi;
		double yf;
	};

	LinearParameters params_;

	LinearPointConstPtr from_;
	LinearPointConstPtr to_;
	bool saturation_;
};

using LinearInterpolatorPtr = std::shared_ptr<LinearInterpolator>;
using LinearInterpolatorConstPtr = std::shared_ptr<const LinearInterpolator>;

} // namespace RSCL
