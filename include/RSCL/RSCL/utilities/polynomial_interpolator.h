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
 * @file polynomial_interpolator.h
 * @author Benjamin Navarro
 * @brief Definition of the PolynomialPoint struct and the PolynomialInterpolator class
 * @date April 2017
 * @ingroup RSCL
 */

#pragma once

#include <RSCL/utilities/interpolator.h>
#include <RSCL/utilities/interpolators_common.h>
#include <RSCL/utilities/fifth_order_polynomial.h>

namespace RSCL {

/** @brief A fifth order polynomial interpolator.
 *  @details This interpolator is useful when you have constraints on the first and second derivatives at the starting and ending points.
 *  The output is constrained to the [from.y to.y] range.
 */
class PolynomialInterpolator : public Interpolator {
public:
	/**
	 * @brief Construct a polynomial interpolator given starting and ending points and a user defined input
	 * @param from Starting point.
	 * @param to Ending point.
	 * @param input A shared pointer to the input value used by the interpolator.
	 */
	PolynomialInterpolator(PolynomialPointConstPtr from, PolynomialPointConstPtr to, doubleConstPtr input);

	/**
	 * @brief Construct a polynomial interpolator given starting and ending points.
	 * The input pointer can be set with Interpolator::setInput. Maily useful when embedded in an other class.
	 * @param from Starting point.
	 * @param to Ending point.
	 */
	PolynomialInterpolator(PolynomialPointConstPtr from, PolynomialPointConstPtr to);

	~PolynomialInterpolator() = default;

	virtual void computeParameters() override;
	virtual double compute() override;

private:
	FifthOrderPolynomial::Parameters params_;

	PolynomialPointConstPtr from_;
	PolynomialPointConstPtr to_;
};

using PolynomialInterpolatorPtr = std::shared_ptr<PolynomialInterpolator>;
using PolynomialInterpolatorConstPtr = std::shared_ptr<const PolynomialInterpolator>;

} // namespace RSCL
