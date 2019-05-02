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

} // namespace phri
