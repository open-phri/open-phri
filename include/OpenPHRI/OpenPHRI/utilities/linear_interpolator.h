/*      File: linear_interpolator.h
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
 * @file linear_interpolator.h
 * @author Benjamin Navarro
 * @brief Definition of the LinearInterpolator class
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/utilities/interpolator.h>
#include <OpenPHRI/utilities/interpolators_common.h>

namespace phri {

/** @brief A linear interpolator with an optional saturation.
 */
template <typename X, typename Y>
class LinearInterpolator : public Interpolator<X, Y, LinearInterpolator<X, Y>> {
public:
    struct Point {
        Point(const X& x, const Y& y) : x{x}, y{y} {
        }

        X x; // x value
        Y y; // y value
    };

    LinearInterpolator(const Point& from, const Point& to)
        : from_{from}, to_{to} {
        computeParameters();
    }

    /**
     * @brief Construct a linear interpolator given starting and ending points
     * and a user defined input
     * @param from Starting point.
     * @param to Ending point.
     * @param input A shared pointer to the input value used by the
     * interpolator.
     */
    LinearInterpolator(const Point& from, const Point& to,
                       std::shared_ptr<const double> input)
        : LinearInterpolator{from, to} {
        this->setInput(input);
    }

    /**
     * @brief Turn the saturation on or off. If on, the output will be
     * restricted to the [from.y() to.y()] range.
     * @param on True to enable the saturation, false otherwise.
     */
    void enableSaturation(bool on) {
        saturation_ = on;
    }

    void computeParameters() {
        params_.xi = static_cast<double>(from().x);
        params_.xf = static_cast<double>(to().x);
        params_.yi = static_cast<double>(from().y);
        params_.yf = static_cast<double>(to().y);

        params_.a = (params_.yf - params_.yi) / (params_.xf - params_.xi);
        params_.b = params_.yi - params_.a * params_.xi;
    }

    const Y& compute() {
        const double& x = static_cast<const double&>(*this->input_);
        double& y = static_cast<double&>(*this->output_);

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

        return *this->output_;
    }

    const Point& from() const {
        return from_;
    }

    Point& from() {
        return from_;
    }

    const Point& to() const {
        return to_;
    }

    Point& to() {
        return to_;
    }

private:
    struct LinearParameters {
        double a{0};
        double b{0};

        double xi{0};
        double xf{0};
        double yi{0};
        double yf{0};
    };

    LinearParameters params_;

    Point from_;
    Point to_;
    bool saturation_{false};
};

} // namespace phri
