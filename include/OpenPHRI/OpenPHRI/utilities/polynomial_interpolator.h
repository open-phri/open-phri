/*      File: polynomial_interpolator.h
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
 * @file polynomial_interpolator.h
 * @author Benjamin Navarro
 * @brief Definition of the PolynomialPoint struct and the
 * PolynomialInterpolator class
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/utilities/interpolator.h>
#include <OpenPHRI/utilities/interpolators_common.h>
#include <OpenPHRI/utilities/fifth_order_polynomial.h>

namespace phri {

/** @brief A fifth order polynomial interpolator.
 *  @details This interpolator is useful when you have constraints on the first
 * and second derivatives at the starting and ending points. The output is
 * constrained to the [from.y to.y] range.
 */
class PolynomialInterpolator : public Interpolator {
public:
    /**
     * @brief Construct a polynomial interpolator given starting and ending
     * points and a user defined input
     * @param from Starting point.
     * @param to Ending point.
     * @param input A shared pointer to the input value used by the
     * interpolator.
     */
    PolynomialInterpolator(std::shared_ptr<const PolynomialPoint> from,
                           std::shared_ptr<const PolynomialPoint> to,
                           std::shared_ptr<const double> input);

    /**
     * @brief Construct a polynomial interpolator given starting and ending
     * points. The input pointer can be set with Interpolator::setInput. Maily
     * useful when embedded in an other class.
     * @param from Starting point.
     * @param to Ending point.
     */
    PolynomialInterpolator(std::shared_ptr<const PolynomialPoint> from,
                           std::shared_ptr<const PolynomialPoint> to);

    ~PolynomialInterpolator() = default;

    virtual void computeParameters() override;
    virtual double compute() override;

private:
    FifthOrderPolynomial::Parameters params_;

    std::shared_ptr<const PolynomialPoint> from_;
    std::shared_ptr<const PolynomialPoint> to_;
};

} // namespace phri
