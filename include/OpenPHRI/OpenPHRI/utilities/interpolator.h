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
template <typename X, typename Y, typename ParentT> class Interpolator {
public:
    Interpolator()
        : input_{std::make_shared<X>(0.)}, output_{std::make_shared<Y>(0.)} {
    }

    /**
     * @brief Get the pointer to the output data.
     * @return A shared pointer to the output data.
     */
    [[nodiscard]] std::shared_ptr<const Y> outputPtr() const {
        return output_;
    }

    /**
     * @brief Get the pointer to the output data.
     * @return A shared pointer to the output data.
     */
    [[nodiscard]] const Y& output() const {
        return *output_;
    }

    /**
     * @brief Set the pointer to the input data.
     * @return A shared pointer to the input data.
     */
    void setInput(std::shared_ptr<const X> input) {
        input_ = input;
    }

    /**
     * @brief Set the pointer to the input data.
     * @return A shared pointer to the input data.
     */
    void setInput(const X* input) {
        input_ = std::shared_ptr<const X>(input, [](auto p) {});
    }

    /**
     * @brief Call operator, shortcut for compute()
     * @return The new output data.
     */
    const Y& operator()() {
        auto& self = static_cast<ParentT&>(*this);
        self.compute();
    }

protected:
    std::shared_ptr<const X> input_;
    std::shared_ptr<Y> output_;
};

} // namespace phri
