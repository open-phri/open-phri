/*      File: force_generator.h
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
 * @file force_generator.h
 * @author Benjamin Navarro
 * @brief Base class definition for force generators
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/robot.h>
#include <OpenPHRI/fwd_decl.h>

namespace phri {

/** @brief Base class for all force generators.
 *  @details Provides a pure virtual compute method.
 */
class ForceGenerator {
public:
    virtual ~ForceGenerator() = default;

    /**
     * @brief Compute the value associated with the force generator.
     * @return The force generator's evaluated value.
     */
    virtual Vector6d compute() final;

    /**
     * @brief Call operator, shortcut for compute().
     * @return The force generator's evaluated value.
     */
    virtual Vector6d operator()() final;

protected:
    friend class SafetyController;

    /**
     * @brief Construct a force generator
     * @param frame The reference frame in which the force is expressed.
     */
    explicit ForceGenerator(ReferenceFrame frame);

    /**
     * @brief Transform the given force in the TCP frame, if necessary.
     * @param force The force to transform.
     */
    virtual Vector6d transform(const Vector6d& force) final;

    virtual void update(Vector6d& force) = 0;

    /**
     * @brief Set the robot to work with.
     * @param robot The robot.
     */
    virtual void setRobot(Robot const* robot);

    Robot const* robot_;
    ReferenceFrame frame_;
};

using ForceGeneratorPtr = std::shared_ptr<ForceGenerator>;
using ForceGeneratorConstPtr = std::shared_ptr<const ForceGenerator>;

} // namespace phri
