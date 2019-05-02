/*      File: polynomial_interpolator.cpp
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

#include <OpenPHRI/utilities/polynomial_interpolator.h>

using namespace phri;

PolynomialInterpolator::PolynomialInterpolator(PolynomialPointConstPtr from,
                                               PolynomialPointConstPtr to,
                                               doubleConstPtr input)
    : PolynomialInterpolator(from, to) {
    setInput(input);
}

PolynomialInterpolator::PolynomialInterpolator(PolynomialPointConstPtr from,
                                               PolynomialPointConstPtr to) {
    from_ = from;
    to_ = to;
    output_ = std::make_shared<double>(0.);

    computeParameters();
}

void PolynomialInterpolator::computeParameters() {
    double xi = *from_->x, xf = *to_->x;
    double yi = *from_->y, yf = *to_->y;
    double dyi = *from_->dy, dyf = *to_->dy;
    double d2yi = *from_->d2y, d2yf = *to_->d2y;
    params_ = {xi, xf, yi, yf, dyi, dyf, d2yi, d2yf};
    FifthOrderPolynomial::computeParameters(params_);
}

double PolynomialInterpolator::compute() {
    *output_ = FifthOrderPolynomial::compute(*input_, params_);

    return *output_;
}
