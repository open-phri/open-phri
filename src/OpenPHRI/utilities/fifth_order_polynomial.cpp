/*      File: fifth_order_polynomial.cpp
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

#include <OpenPHRI/utilities/fifth_order_polynomial.h>

#include "polynomial_root_finder.h"
#include <iostream>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace phri {

size_t FifthOrderPolynomial::compute_timings_total_iter = 0;

/* Simplified coefficient for xi = 0 and dx = xf-xi
 * a = -(12*yi - 12*yf + 6*dx*dyf + 6*dx*dyi - d2yf*dx^2 + d2yi*dx^2)/(2*dx^5)
 * b = (30*yi - 30*yf + 14*dx*dyf + 16*dx*dyi - 2*d2yf*dx^2 +
 * 3*d2yi*dx^2)/(2*dx^4) c = -(20*yi - 20*yf + 8*dx*dyf + 12*dx*dyi - d2yf*dx^2
 * + 3*d2yi*dx^2)/(2*dx^3) d = d2yi/2 e = dyi f = yi
 */
void FifthOrderPolynomial::computeParameters(
    FifthOrderPolynomial::Parameters& params) {

    double dx = params.xf - params.xi;
    double dx_2 = dx * dx, dx_3 = dx_2 * dx, dx_4 = dx_3 * dx, dx_5 = dx_4 * dx;

    params.a =
        -(12. * params.yi - 12. * params.yf + 6. * dx * params.dyf +
          6. * dx * params.dyi - params.d2yf * dx_2 + params.d2yi * dx_2) /
        (2. * dx_5);
    params.b = (30. * params.yi - 30. * params.yf + 14. * dx * params.dyf +
                16. * dx * params.dyi - 2. * params.d2yf * dx_2 +
                3. * params.d2yi * dx_2) /
               (2. * dx_4);
    params.c = -(20. * params.yi - 20. * params.yf + 8. * dx * params.dyf +
                 12. * dx * params.dyi - params.d2yf * dx_2 +
                 3. * params.d2yi * dx_2) /
               (2. * dx_3);
    params.d = params.d2yi / 2.;
    params.e = params.dyi;
    params.f = params.yi;
}

FifthOrderPolynomial::ConstraintError
FifthOrderPolynomial::computeParametersWithConstraints(
    FifthOrderPolynomial::Parameters& parameters, double dymax, double d2ymax,
    double dyeps, double d2yeps, double initial_guess) {
    ConstraintError error = FifthOrderPolynomial::ConstraintError::NoError;
    if (dymax < std::abs(parameters.dyi)) {
        error = static_cast<ConstraintError>(error |
                                             ConstraintError::InitialVelocity);
    }
    if (dymax < std::abs(parameters.dyf)) {
        error = static_cast<ConstraintError>(error |
                                             ConstraintError::FinalVelocity);
    }
    if (d2ymax < std::abs(parameters.d2yi)) {
        error = static_cast<ConstraintError>(
            error | ConstraintError::InitialAcceleration);
    }
    if (d2ymax < std::abs(parameters.d2yf)) {
        error = static_cast<ConstraintError>(
            error | ConstraintError::FinalAcceleration);
    }

    if (error != FifthOrderPolynomial::ConstraintError::NoError) {
        return error;
    }

    FifthOrderPolynomial::Parameters poly_params_dy =
        FifthOrderPolynomial::Constraints{0,
                                          initial_guess,
                                          parameters.yi,
                                          parameters.yf,
                                          parameters.dyi,
                                          parameters.dyf,
                                          parameters.d2yi,
                                          parameters.d2yf};
    FifthOrderPolynomial::Parameters poly_params_d2y =
        FifthOrderPolynomial::Constraints{0,
                                          initial_guess,
                                          parameters.yi,
                                          parameters.yf,
                                          parameters.dyi,
                                          parameters.dyf,
                                          parameters.d2yi,
                                          parameters.d2yf};

    if (parameters.yi == parameters.yf) {
        poly_params_dy.xf = std::numeric_limits<double>::min();
        FifthOrderPolynomial::computeParameters(poly_params_dy);
        parameters = poly_params_dy;
        return error;
    }

    bool dymax_found = false;
    bool d2ymax_found = false;
    double v_max = 0., a_max = 0.;
    auto get_dymax_error = [&poly_params_dy, dymax, &v_max]() {
        v_max = FifthOrderPolynomial::getFirstDerivativeMaximum(poly_params_dy);
        return v_max - dymax;
    };
    auto get_d2ymax_error = [&poly_params_d2y, d2ymax, &a_max]() {
        a_max =
            FifthOrderPolynomial::getSecondDerivativeMaximum(poly_params_d2y);
        return a_max - d2ymax;
    };

    while (not(dymax_found and d2ymax_found)) {

        if (not dymax_found) {
            FifthOrderPolynomial::computeParameters(poly_params_dy);
            ++FifthOrderPolynomial::compute_timings_total_iter;
            double v_error_abs = std::abs(get_dymax_error());
            dymax_found = v_error_abs < dyeps;
            if (not dymax_found) {
                poly_params_dy.xf = poly_params_dy.xf * v_max / dymax;
            }
        }
        if (not d2ymax_found) {
            FifthOrderPolynomial::computeParameters(poly_params_d2y);
            ++FifthOrderPolynomial::compute_timings_total_iter;
            double a_error_abs = std::abs(get_d2ymax_error());
            d2ymax_found = a_error_abs < d2yeps;
            if (not d2ymax_found) {
                poly_params_d2y.xf =
                    poly_params_d2y.xf * (std::sqrt(a_max / d2ymax));
            }
        }
    }

    if (poly_params_dy.xf > poly_params_d2y.xf) {
        parameters = poly_params_dy;
    } else {
        parameters = poly_params_d2y;
    }

    return error;
}

double FifthOrderPolynomial::compute(double x, const Parameters& params) {
    double y;

    if (x < params.xi) {
        y = params.yi;
    } else if (x > params.xf) {
        y = params.yf;
    } else {
        x -= params.xi;
        double x_2 = x * x, x_3 = x_2 * x, x_4 = x_3 * x, x_5 = x_4 * x;
        y = params.a * x_5 + params.b * x_4 + params.c * x_3 + params.d * x_2 +
            params.e * x + params.f;
    }

    return y;
}

double FifthOrderPolynomial::computeFirstDerivative(double x,
                                                    const Parameters& params) {
    double dy;

    if (x < params.xi) {
        dy = params.dyi;
    } else if (x > params.xf) {
        dy = params.dyf;
    } else {
        x -= params.xi;
        double x_2 = x * x, x_3 = x_2 * x, x_4 = x_3 * x;
        dy = 5. * params.a * x_4 + 4. * params.b * x_3 + 3. * params.c * x_2 +
             2. * params.d * x + params.e;
    }

    return dy;
}

double FifthOrderPolynomial::computeSecondDerivative(double x,
                                                     const Parameters& params) {
    double d2y;

    if (x < params.xi) {
        d2y = params.d2yi;
    } else if (x > params.xf) {
        d2y = params.d2yf;
    } else {
        x -= params.xi;
        double x_2 = x * x, x_3 = x_2 * x;
        d2y = 20. * params.a * x_3 + 12. * params.b * x_2 + 6. * params.c * x +
              2. * params.d;
    }

    return d2y;
}

double
FifthOrderPolynomial::getFirstDerivativeMaximum(const Parameters& params) {
    int degree = 3;
    double coeffs[MDP1] = {20. * params.a, 12. * params.b, 6. * params.c,
                           2. * params.d};
    double roots_real[MAXDEGREE];
    double roots_img[MAXDEGREE];

    rpoly_ak1(coeffs, &degree, roots_real, roots_img);
    if (degree == 0) {
        throw std::runtime_error(
            "In FifthOrderPolynomial::getFirstDerivativeMaximum, rpoly_ak1 "
            "failed");
    }

    auto poly = [&params](double x) -> double {
        double x_2 = x * x, x_3 = x_2 * x, x_4 = x_3 * x;
        return std::abs(5. * params.a * x_4 + 4. * params.b * x_3 +
                        3. * params.c * x_2 + 2. * params.d * x + params.e);
    };

    double max = 0.;
    for (size_t i = 0; i < 3; ++i) {
        max = std::max(max, poly(roots_real[i]));
    }

    return max;
}

double
FifthOrderPolynomial::getSecondDerivativeMaximum(const Parameters& params) {
    int degree = 2;
    double coeffs[MDP1] = {60. * params.a, 24. * params.b, 6. * params.c};
    double roots_real[MAXDEGREE];
    double roots_img[MAXDEGREE];

    rpoly_ak1(coeffs, &degree, roots_real, roots_img);
    if (degree == 0) {
        throw std::runtime_error(
            "In FifthOrderPolynomial::getSecondDerivativeMaximum, rpoly_ak1 "
            "failed");
    }

    auto poly = [&params](double x) -> double {
        double x_2 = x * x, x_3 = x_2 * x;
        return std::abs(20. * params.a * x_3 + 12. * params.b * x_2 +
                        6. * params.c * x + 2. * params.d);
    };

    double max = std::max(poly(roots_real[0]), poly(roots_real[1]));

    return max;
}

} // namespace phri
