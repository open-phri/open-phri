/*      File: derivator.hpp
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
 * @file derivator.hpp
 * @author Benjamin Navarro
 * @brief Generir first order derivator
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/detail/universal_wrapper.hpp>
#include <physical_quantities/spatial/velocity.h>
#include <physical_quantities/spatial/acceleration.h>

namespace phri {

/** @brief A generic derivator class.
 *  @details Works on primitive types and Eigen vectors
 *  @tparam Input The input type to work on
 *  @tparam Output The output type to produce
 */
template <typename Input, typename Output = Input> class Derivator {
public:
    /***		Constructor & destructor		***/

    /**
     * @brief Construct a Derivator for a given input and sample time.
     * @param input A shared pointer to the input data
     * @param output A shared pointer to the output data
     * @param sample_time Time step between each call to Derivator::compute().
     */
    template <typename InputT, typename OutputT>
    Derivator(InputT&& input, OutputT&& output, double sample_time)
        : input_{std::forward<InputT>(input)},
          output_{std::forward<OutputT>(output)},
          frequency_{1. / sample_time} {
        reset();
    }

    /**
     * @brief Construct a Derivator for a given input and sample time.
     * @param input A shared pointer to the input data
     * @param sample_time Time step between each call to Derivator::compute().
     */
    template <typename InputT>
    Derivator(InputT&& input, double sample_time)
        : Derivator(std::forward<InputT>(input), Output{}, sample_time) {
    }

    virtual ~Derivator() = default;

    /**
     * @brief Reset the derivator's internal state to its original state
     */
    template <typename TT = Output>
    typename std::enable_if<
        std::is_same<TT,
                     Eigen::Matrix<typename TT::Scalar, TT::RowsAtCompileTime,
                                   TT::ColsAtCompileTime, TT::Options,
                                   TT::MaxRowsAtCompileTime,
                                   TT::MaxColsAtCompileTime>>::value,
        void>::type
    reset() {
        output().setZero();
        previous_input_ = input();
    }

    /**
     * @brief Reset the derivator's internal state to its original state
     */
    template <typename TT = Output>
    typename std::enable_if<std::is_same<TT, spatial::Velocity>::value or
                                std::is_same<TT, spatial::Acceleration>::value,
                            void>::type
    reset() {
        output_().setZero();
        previous_input_ = input();
    }

    /**
     * @brief Reset the derivator's internal state to its original state
     */
    template <typename TT = Output>
    typename std::enable_if<std::is_arithmetic<TT>::value, void>::type reset() {
        output() = TT{0};
        previous_input_ = input();
    }

    /***		Algorithm		***/

    /**
     * @brief Update the derivative estimation.
     * @return The new estimation.
     */
    virtual Output compute() {
        output() = (input() - previous_input_) * frequency_;
        previous_input_ = input();
        return output();
    }

    /**
     * @brief Call operator, shortcut for compute()
     * @return The new estimation.
     */
    Output operator()() {
        return compute();
    }

    const Input& input() const {
        return input_.cref();
    }

    Output& output() {
        return output_.ref();
    }

    const Output& output() const {
        return output_.cref();
    }

private:
    detail::UniversalWrapper<const Input> input_;
    detail::UniversalWrapper<Output> output_;
    Input previous_input_;
    double frequency_;
};

extern template class Derivator<double>;
extern template class Derivator<Eigen::Vector2d>;
extern template class Derivator<Eigen::Vector3d>;
extern template class Derivator<Eigen::Vector6d>;

} // namespace phri
