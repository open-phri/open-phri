/*      File: velocity_generator.h
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

//! \file velocity_generator.h
//! \author Benjamin Navarro
//! \brief Base class for all velocity generators.
//! \date 05-2019
//! \ingroup phri

#pragma once

#include <memory>

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/robot.h>
#include <OpenPHRI/fwd_decl.h>

#include <physical_quantities/spatial/velocity.h>

namespace phri {

//! \brief Base class for all velocity generators.
//! \details Provides a pure virtual compute method to be implemented by derived
//! classes.
class VelocityGenerator {
public:
    //! \brief Default constructor
    VelocityGenerator() = default;

    //! \brief Delete copy constructor to avoid slicing
    VelocityGenerator(const VelocityGenerator&) = delete;

    //! \brief Default move constructor
    VelocityGenerator(VelocityGenerator&&) = default;

    //! \brief Default virtual destructor
    virtual ~VelocityGenerator() = default;

    //! \brief Delete copy operator to avoid slicing
    VelocityGenerator& operator=(const VelocityGenerator&) = delete;

    //! \brief Default move operator
    VelocityGenerator& operator=(VelocityGenerator&&) = default;

    //! \brief Compute the value associated with the velocity generator.
    //! \return The velocity generator's evaluated value.
    [[nodiscard]] const spatial::Velocity& compute();

    //! \brief Call operator, shortcut for compute().
    //! \return The velocity generator's evaluated value.
    [[nodiscard]] const spatial::Velocity& operator()();

    [[nodiscard]] const spatial::Frame& frame() const;

protected:
    friend class SafetyController;

    //! \brief Derived classed must implement this to provide their velocity
    //! output
    //! \param velocity A reference to the velocity to set
    virtual void update(spatial::Velocity& velocity) = 0;

    //! \brief Set the robot to work with.
    //! \param robot The robot.
    virtual void setRobot(Robot const* robot);

    void setFrame(const spatial::Frame& frame);

    //! \brief Read/write access the controlled robot
    //! \return double& A reference to the controlled robot
    [[nodiscard]] const Robot& robot() const;

    //! \brief Read access to the internal velocity
    //! \return Twist The internal velocity
    [[nodiscard]] virtual const spatial::Velocity& internalVelocity() const;

private:
    Robot const* robot_{nullptr};
    spatial::Velocity velocity_;
};

} // namespace phri
