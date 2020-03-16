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
//! \brief
//! \date 05-2019
//! \ingroup phri

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/robot.h>
#include <OpenPHRI/fwd_decl.h>

#include <physical_quantities/spatial/force.h>

namespace phri {

//! \brief Base class for all force generators.
//! \details Provides a pure virtual compute method to be implemented by derived
//! classes.
class ForceGenerator {
public:
    //! \brief Default constructor
    // ForceGenerator(spatial::Frame frame);
    ForceGenerator() = default;

    //! \brief Delete copy constructor to avoid slicing
    ForceGenerator(const ForceGenerator&) = delete;

    //! \brief Default move constructor
    ForceGenerator(ForceGenerator&&) = default;

    //! \brief Default virtual destructor
    virtual ~ForceGenerator() = default;

    //! \brief Delete copy operator to avoid slicing
    ForceGenerator& operator=(const ForceGenerator&) = delete;

    //! \brief Default move operator
    ForceGenerator& operator=(ForceGenerator&&) = default;

    //! \brief Compute the value associated with the force generator.
    //! \return The force generator's evaluated value.
    const spatial::Force& compute();

    //! \brief Call operator, shortcut for compute().
    //! \return The force generator's evaluated value.
    const spatial::Force& operator()();

    const spatial::Frame& frame() const;

protected:
    friend class SafetyController;

    //! \brief Derived classed must implement this to provide their wrench
    //! output
    //! \param wrench A reference to the wrench to set
    virtual void update(spatial::Force& wrench) = 0;

    //! \brief Set the robot to work with.
    //! \param robot The robot.
    virtual void setRobot(Robot const* robot);

    //! \brief Read only access the controlled robot
    //! \return double& A reference to the controlled robot
    const Robot& robot() const;

    void setFrame(const spatial::Frame& frame);

    //! \brief Read access to the internal wrench
    //! \return ReferenceFrame The internal wrench
    const spatial::Force& internalWrench() const;

private:
    Robot const* robot_;
    spatial::Force wrench_;
};

} // namespace phri
