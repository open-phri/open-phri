/*      File: deadband.hpp
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
 * @file deadband.h
 * @author Benjamin Navarro
 * @brief Definition of the Deandband class
 * @date July 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/definitions.h>

namespace phri {

template <class T> class Deadband {
public:
    Deadband(const std::shared_ptr<T>& data,
             const std::shared_ptr<const T>& threshold)
        : data_in_(data), data_out_(data), threshold_(threshold) {
    }
    Deadband(const std::shared_ptr<const T>& data_in,
             const std::shared_ptr<T>& data_out,
             const std::shared_ptr<const T>& threshold)
        : data_in_(data_in), data_out_(data_out) {
    }
    ~Deadband() = default;

    void compute() const {
        const auto& in = *data_in_;
        auto& out = *data_out_;
        const auto& th = *threshold_;
        for (size_t i = 0; i < data_in_->size(); ++i) {
            if (in[i] > th[i]) {
                out[i] = in[i] - th[i];
            } else if (in[i] < -th[i]) {
                out[i] = in[i] + th[i];
            } else {
                out[i] = 0.;
            }
        }
    }

    void operator()() const {
        compute();
    }

private:
    std::shared_ptr<const T> data_in_;
    std::shared_ptr<T> data_out_;
    std::shared_ptr<const T> threshold_;
};

extern template class Deadband<phri::Vector3d>;
extern template class Deadband<phri::Vector6d>;
extern template class Deadband<phri::VectorXd>;

} // namespace phri
