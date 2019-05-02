/*      File: double_wrappers.h
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

#pragma once

#include <memory>

// Primitive types need to be wrapped in order to use pointers on them in python
struct DoubleWrap {
    DoubleWrap(double init_value = 0.)
        : value(std::make_shared<double>(init_value)) {
    }

    DoubleWrap(std::shared_ptr<double> ptr) : value(ptr) {
    }

    operator std::shared_ptr<double>() const {
        return value;
    }

    operator std::shared_ptr<const double>() const {
        return value;
    }

    void set(double val) {
        *value = val;
    }

    double get() const {
        return *value;
    }

private:
    std::shared_ptr<double> value;
};

struct ConstDoubleWrap {
    ConstDoubleWrap(std::shared_ptr<const double> ptr) : value(ptr) {
    }

    operator std::shared_ptr<const double>() const {
        return value;
    }

    double get() const {
        return *value;
    }

private:
    std::shared_ptr<const double> value;
};
