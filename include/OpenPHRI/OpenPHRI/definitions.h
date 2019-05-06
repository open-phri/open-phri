/*      File: definitions.h
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
 * @file definitions.h
 * @author Benjamin Navarro
 * @brief Usefull definitions being used in OpenPHRI
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

// Select only the units useful to OpenPHRI to improve compilation times
#define DISABLE_PREDEFINED_UNITS
#define ENABLE_PREDEFINED_FREQUENCY_UNITS
#define ENABLE_PREDEFINED_TIME_UNITS
#include <OpenPHRI/units.h>

#include <OpenPHRI/type_aliases.h>
#include <OpenPHRI/task_space_data_types.h>

#include <memory>
#include <type_traits>

namespace phri {

#define assert_msg(msg, cond) assert(((void)msg, cond))

/** @enum phri::ReferenceFrame
 *  @brief Specify an object's reference frame.
 */
enum class ReferenceFrame {
    TCP,  /**< Controlled frame. Tool control frame */
    Base, /**< Frame fixed relative to the robot's base */
    World /**< Frame fixed relative to the environment */
};

template <class T> struct is_shared_ptr : std::false_type {};

template <class T> struct is_shared_ptr<std::shared_ptr<T>> : std::true_type {};

//! \brief Forms lvalue reference to const type of t
//! \details Same as C++17 std::as_const. Only for C++14 compability.
//! \tparam T Type to cast
//! \param t Value to cast
//! \return constexpr std::add_const_t<T>& a const lvalue reference to \p t
template <class T> constexpr std::add_const_t<T>& as_const(T& t) noexcept {
    return t;
}

} // namespace phri
