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

//! \file force_generator.h
//! \author Benjamin Navarro
//! \brief Base class for all force generators.
//! \date 05-2019
//! \ingroup phri

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/robot.h>
#include <OpenPHRI/fwd_decl.h>

#include <physical_quantities/vector/force.h>

namespace phri {

//! \brief Base class for all force generators.
//! \details Provides a pure virtual compute method to be implemented by derived
//! classes.
class JointForceGenerator {
public:
    JointForceGenerator() = default;

    //! \brief Delete copy constructor to avoid slicing
    JointForceGenerator(const JointForceGenerator&) = delete;

    //! \brief Default move constructor
    JointForceGenerator(JointForceGenerator&&) = default;

    //! \brief Default virtual destructor
    virtual ~JointForceGenerator() = default;

    //! \brief Delete copy operator to avoid slicing
    JointForceGenerator& operator=(const JointForceGenerator&) = delete;

    //! \brief Default move operator
    JointForceGenerator& operator=(JointForceGenerator&&) = default;

    /**
     * @brief Compute the value associated with the force generator.
     * @return The force generator's evaluated value.
     */
    const vector::dyn::Force& compute();

    /**
     * @brief Call operator, shortcut for compute().
     * @return The force generator's evaluated value.
     */
    const vector::dyn::Force& operator()();

protected:
    friend class SafetyController;

    virtual void update(vector::dyn::Force& force) = 0;

    /**
     * @brief Set the robot to work with.
     * @param robot The robot.
     */
    virtual void setRobot(Robot const* robot);

    //! \brief Read/write access the controlled robot
    //! \return double& A reference to the controlled robot
    const Robot& robot() const;

    Robot const* robot_;

    vector::dyn::Force force_;
};

} // namespace phri
