/*      File: integrator.hpp
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
 * @file integrator.hpp
 * @author Benjamin Navarro
 * @brief Generir first order integrator
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/definitions.h>
#include <physical_quantities/spatial/type_aliases.h>

namespace phri {

/** @brief A generic integrator class.
 *  @details Works on any type that provides '=' and '+=' operators. Principally
 * used on primitive types and Eigen vectors.
 *  @tparam T The type to work on
 */
template <typename T> class Integrator {
public:
    /***		Constructor & destructor		***/

    /**
     * @brief Construct an Integrator for a given input and sample time.
     * @param input A shared pointer to the input data
     * @param output A shared pointer to the output data
     * @param sample_time Time step between each call to Integrator::compute().
     */
    Integrator(const std::shared_ptr<const T>& input,
               const std::shared_ptr<T>& output, double sample_time)
        : input_(input), output_(output), sample_time_(sample_time) {
        reset();
    }

    /**
     * @brief Construct an Integrator for a given input and sample time.
     * @param input A shared pointer to the input data
     * @param sample_time Time step between each call to Integrator::compute().
     */
    Integrator(const std::shared_ptr<const T>& input, double sample_time)
        : Integrator(input, std::make_shared<T>(), sample_time) {
    }

    virtual ~Integrator() = default;

    /***		Configuration		***/

    /**
     * @brief Get the pointer to the output data
     * @return A shared pointer to the output data.
     */
    std::shared_ptr<const T> getOutput() const {
        return output_;
    }

    /**
     * @brief Reset the integrators's output to the current input.
     */
    void reset() {
        *output_ = *input_;
    }

    /**
     * @brief Set the integrators's output to the given value.
     * @param value A shared pointer to the value to use
     */
    void force(const std::shared_ptr<const T>& value) {
        *output_ = *value;
    }

    /**
     * @brief Set the integrators's output to the given value.
     * @param value The value to use
     */
    void force(const T& value) {
        *output_ = value;
    }

    /***		Algorithm		***/

    /**
     * @brief Update the integrator.
     * @return The new integrator output.
     */
    virtual T compute() {
        *output_ += *input_ * sample_time_;
        return *output_;
    }

    /**
     * @brief Call operator, shortcut for compute().
     * @return The new integrator output.
     */
    T operator()() {
        return compute();
    }

private:
    std::shared_ptr<const T> input_;
    std::shared_ptr<T> output_;
    double sample_time_;
};

template <typename T> using IntegratorPtr = std::shared_ptr<Integrator<T>>;
template <typename T>
using IntegratorConstPtr = std::shared_ptr<const Integrator<T>>;

extern template class Integrator<double>;
extern template class Integrator<Eigen::Vector2d>;
extern template class Integrator<Eigen::Vector3d>;
extern template class Integrator<Eigen::Vector6d>;

} // namespace phri
