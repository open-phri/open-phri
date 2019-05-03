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
    Derivator(const std::shared_ptr<const Input>& input,
              const std::shared_ptr<Output>& output, double sample_time)
        : input_(input), output_(output), frequency_(1. / sample_time) {
        reset();
    }

    /**
     * @brief Construct a Derivator for a given input and sample time.
     * @param input A shared pointer to the input data
     * @param sample_time Time step between each call to Derivator::compute().
     */
    Derivator(const std::shared_ptr<const Input>& input, double sample_time)
        : Derivator(input, std::make_shared<Output>(), sample_time) {
    }

    virtual ~Derivator() = default;

    /***		Configuration		***/

    /**
     * @brief Get the pointer to the output data
     * @return A shared pointer to the output data.
     */
    std::shared_ptr<const Output> getOutput() const {
        return output_;
    }

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
        output_->setZero();
        previous_input_ = *input_;
    }

    /**
     * @brief Reset the derivator's internal state to its original state
     */
    template <typename TT = Output>
    typename std::enable_if<std::is_same<TT, phri::Twist>::value or
                                std::is_same<TT, phri::Acceleration>::value,
                            void>::type
    reset() {
        output_->vector().setZero();
        previous_input_ = *input_;
    }

    /**
     * @brief Reset the derivator's internal state to its original state
     */
    template <typename TT = Output>
    typename std::enable_if<std::is_arithmetic<TT>::value, void>::type reset() {
        *output_ = TT(0);
        previous_input_ = *input_;
    }

    /***		Algorithm		***/

    /**
     * @brief Update the derivative estimation.
     * @return The new estimation.
     */
    virtual Output compute() {
        *output_ = (*input_ - previous_input_) * frequency_;
        previous_input_ = *input_;
        return *output_;
    }

    /**
     * @brief Call operator, shortcut for compute()
     * @return The new estimation.
     */
    virtual Output operator()() final {
        return compute();
    }

private:
    std::shared_ptr<const Input> input_;
    std::shared_ptr<Output> output_;
    Input previous_input_;
    double frequency_;
};

template <typename T> using DerivatorPtr = std::shared_ptr<Derivator<T>>;
template <typename T>
using DerivatorConstPtr = std::shared_ptr<const Derivator<T>>;

extern template class Derivator<double>;
extern template class Derivator<Vector2d>;
extern template class Derivator<Vector3d>;
extern template class Derivator<Vector6d>;

} // namespace phri
