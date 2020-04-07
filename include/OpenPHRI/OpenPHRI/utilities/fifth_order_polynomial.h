/*      File: fifth_order_polynomial.h
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
 * @file fifth_order_polynomial.h
 * @author Benjamin Navarro
 * @brief Definition of FifthOrderPolynomial class
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <cstddef>

namespace phri {

class FifthOrderPolynomial {
public:
    struct Constraints {
        double xi;
        double xf;
        double yi;
        double yf;
        double dyi;
        double dyf;
        double d2yi;
        double d2yf;
    };

    struct Coefficients {
        double a;
        double b;
        double c;
        double d;
        double e;
        double f;
    };

    struct Parameters : public Constraints, public Coefficients {
        Parameters() = default;

        Parameters(const Constraints& constraints) {
            *this = constraints;
        }

        Parameters(const Coefficients& coefficients) {
            *this = coefficients;
        }

        Parameters(const Constraints& constraints,
                   const Coefficients& coefficients) {
            *this = constraints;
            *this = coefficients;
        }

        Parameters& operator=(const Constraints& other) {
            xi = other.xi;
            xf = other.xf;
            yi = other.yi;
            yf = other.yf;
            dyi = other.dyi;
            dyf = other.dyf;
            d2yi = other.d2yi;
            d2yf = other.d2yf;

            return *this;
        }

        Parameters& operator=(const Coefficients& other) {
            a = other.a;
            b = other.b;
            c = other.c;
            d = other.d;
            e = other.e;
            f = other.f;

            return *this;
        }
    };

    enum ConstraintError {
        NoError = 0,
        InitialVelocity = 1 << 0,
        FinalVelocity = 1 << 1,
        InitialAcceleration = 1 << 2,
        FinalAcceleration = 1 << 3
    };

    static void computeParameters(FifthOrderPolynomial::Parameters& params);
    [[nodiscard]] static ConstraintError computeParametersWithConstraints(
        FifthOrderPolynomial::Parameters& parameters, double dymax,
        double d2ymax, double dyeps, double d2yeps, double initial_guess = 1);
    [[nodiscard]] static double compute(double x, const Parameters& params);
    [[nodiscard]] static double
    computeFirstDerivative(double x, const Parameters& params);
    [[nodiscard]] static double
    computeSecondDerivative(double x, const Parameters& params);
    [[nodiscard]] static double
    getFirstDerivativeMaximum(const Parameters& params);
    [[nodiscard]] static double
    getSecondDerivativeMaximum(const Parameters& params);

    static size_t compute_timings_total_iter;

private:
    FifthOrderPolynomial() = default;
    ~FifthOrderPolynomial() = default;
};

} // namespace phri
