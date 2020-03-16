/*      File: linear_interpolator.cpp
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

#include <OpenPHRI/utilities/linear_interpolator.h>

using namespace phri;

LinearInterpolator::LinearInterpolator(std::shared_ptr<const LinearPoint> from,
                                       std::shared_ptr<const LinearPoint> to,
                                       std::shared_ptr<const double> input)
    : LinearInterpolator(from, to) {
    setInput(input);
}

LinearInterpolator::LinearInterpolator(std::shared_ptr<const LinearPoint> from,
                                       std::shared_ptr<const LinearPoint> to) {
    from_ = from;
    to_ = to;
    output_ = std::make_shared<double>(0.);
    saturation_ = false;

    computeParameters();
}

void LinearInterpolator::enableSaturation(bool on) {
    saturation_ = on;
}

void LinearInterpolator::computeParameters() {
    params_.xi = *from_->x;
    params_.xf = *to_->x;
    params_.yi = *from_->y;
    params_.yf = *to_->y;

    params_.a = (params_.yf - params_.yi) / (params_.xf - params_.xi);
    params_.b = params_.yi - params_.a * params_.xi;
}

double LinearInterpolator::compute() {
    double x = *input_;
    double y;

    if (saturation_) {
        if (x < params_.xi) {
            y = params_.yi;
        } else if (x > params_.xf) {
            y = params_.yf;
        } else {
            y = params_.a * x + params_.b;
        }
    } else {
        y = params_.a * x + params_.b;
    }

    *output_ = y;
    return y;
}
