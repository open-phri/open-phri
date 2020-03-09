/*      File: interpolator.h
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
 * @file interpolator.h
 * @author Benjamin Navarro
 * @brief Base class definition for interpolators
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/definitions.h>

namespace phri {

/** @brief Base class for all interpolators.
 *  @details Provides two pure virtual method to update the internal parameters
 * and to compute a new value.
 */
class Interpolator {
public:
    Interpolator() = default;
    virtual ~Interpolator() = default;

    /**
     * @brief Get the pointer to the output data.
     * @return A shared pointer to the output data.
     */
    std::shared_ptr<const double> getOutput() const;

    /**
     * @brief Set the pointer to the input data.
     * @return A shared pointer to the input data.
     */
    void setInput(std::shared_ptr<const double> input);

    /**
     * @brief Compute the interpolator's parameters .
     */
    virtual void computeParameters() = 0;

    /**
     * @brief Compute the new interpolator output.
     * @return The new output data.
     */
    virtual double compute() = 0;

    /**
     * @brief Call operator, shortcut for compute()
     * @return The new output data.
     */
    double operator()() {
        return compute();
    }

protected:
    std::shared_ptr<const double> input_;
    std::shared_ptr<double> output_;
};

using InterpolatorPtr = std::shared_ptr<Interpolator>;
using InterpolatorConstPtr = std::shared_ptr<const Interpolator>;

} // namespace phri
