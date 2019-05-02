/*      File: torque_generator.h
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
 * @file torque_generator.h
 * @author Benjamin Navarro
 * @brief Base class definition for torque generators
 * @date May 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/robot.h>
#include <OpenPHRI/fwd_decl.h>

namespace phri {

/** @brief Base class for all torque generators.
 *  @details Provides a pure virtual compute method.
 */
class TorqueGenerator {
public:
    TorqueGenerator() = default;
    virtual ~TorqueGenerator() = default;

    /**
     * @brief Compute the value associated with the torque generator.
     * @return The torque generator's evaluated value.
     */
    virtual VectorXd compute() final;

    /**
     * @brief Call operator, shortcut for compute().
     * @return The torque generator's evaluated value.
     */
    virtual VectorXd operator()() final;

protected:
    virtual void update(VectorXd& torque) = 0;
    friend class SafetyController;
    RobotConstPtr robot_;
};

using TorqueGeneratorPtr = std::shared_ptr<TorqueGenerator>;
using TorqueGeneratorConstPtr = std::shared_ptr<const TorqueGenerator>;

} // namespace phri
