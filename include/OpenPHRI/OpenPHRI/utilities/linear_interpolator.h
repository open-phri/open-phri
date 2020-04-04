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
class LinearInterpolator : public Interpolator {
public:
    /**
     * @brief Construct a linear interpolator given starting and ending points
     * and a user defined input
     * @param from Starting point.
     * @param to Ending point.
     * @param input A shared pointer to the input value used by the
     * interpolator.
     */
    LinearInterpolator(std::shared_ptr<const LinearPoint> from,
                       std::shared_ptr<const LinearPoint> to,
                       std::shared_ptr<const double> input);

    /**
     * @brief Construct a linear interpolator given starting and ending points.
     * The input pointer can be set with Interpolator::setInput. Maily useful
     * when embedded in an other class.
     * @param from Starting point.
     * @param to Ending point.
     */
    LinearInterpolator(std::shared_ptr<const LinearPoint> from,
                       std::shared_ptr<const LinearPoint> to);

    virtual ~LinearInterpolator() = default;

    /**
     * @brief Turn the saturation on or off. If on, the output will be
     * restricted to the [from.y() to.y()] range.
     * @param on True to enable the saturation, false otherwise.
     */
    void enableSaturation(bool on);

    void computeParameters() override;

    double compute() override;

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

    std::shared_ptr<const LinearPoint> from_;
    std::shared_ptr<const LinearPoint> to_;
    bool saturation_;
};

} // namespace phri
