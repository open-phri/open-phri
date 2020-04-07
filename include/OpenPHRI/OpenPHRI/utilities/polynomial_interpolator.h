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
 * @brief Definition of the PolynomialInterpolator class
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
template <typename X, typename ValueT, typename FirstDerivative = ValueT,
          typename SecondDerivative = ValueT>
class PolynomialInterpolator
    : public Interpolator<X, ValueT,
                          PolynomialInterpolator<X, ValueT, FirstDerivative,
                                                 SecondDerivative>> {
public:
    struct Point
        : public TrajectoryPoint<ValueT, FirstDerivative, SecondDerivative> {
        Point(const X& x, const ValueT& y, const FirstDerivative& dy,
              const SecondDerivative& d2y)
            : TrajectoryPoint<ValueT, FirstDerivative, SecondDerivative>(y, dy,
                                                                         d2y),
              x{x} {
        }

        X x; // x value
    };

    /**
     * @brief Construct a polynomial interpolator given starting and ending
     * points. The input pointer can be set with Interpolator::setInput. Maily
     * useful when embedded in an other class.
     * @param from Starting point.
     * @param to Ending point.
     */
    PolynomialInterpolator(const Point& from, const Point& to)
        : from_{from}, to_{to} {
        computeParameters();
    }

    /**
     * @brief Construct a polynomial interpolator given starting and ending
     * points and a user defined input
     * @param from Starting point.
     * @param to Ending point.
     * @param input A shared pointer to the input value used by the
     * interpolator.
     */
    PolynomialInterpolator(const Point& from, const Point& to,
                           std::shared_ptr<const double> input)
        : PolynomialInterpolator(from, to) {
        this->setInput(input);
    }

    ~PolynomialInterpolator() = default;

    void computeParameters() {
        double xi = from().x, xf = to().x;
        double yi = *from().y, yf = *to().y;
        double dyi = *from().dy, dyf = *to().dy;
        double d2yi = *from().d2y, d2yf = *to().d2y;
        params_ = {xi, xf, yi, yf, dyi, dyf, d2yi, d2yf};
        FifthOrderPolynomial::computeParameters(params_);
    }

    const ValueT& compute() {
        ValueT& y = *this->output_;
        y = FifthOrderPolynomial::compute(static_cast<double>(*this->input_),
                                          params_);
        return y;
    }

    [[nodiscard]] const Point& from() const {
        return from_;
    }

    [[nodiscard]] Point& from() {
        return from_;
    }

    [[nodiscard]] const Point& to() const {
        return to_;
    }

    [[nodiscard]] Point& to() {
        return to_;
    }

private:
    FifthOrderPolynomial::Parameters params_;

    Point from_;
    Point to_;
};

} // namespace phri
