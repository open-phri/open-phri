/*      File: mass_generator.h
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

//! \file mass_generator.h
//! \author Benjamin Navarro
//! \brief Generates a force as if a virtual mass is attached to the robot.
//! \date 05-2019
//! \ingroup phri

#pragma once

#include <OpenPHRI/force_generators/force_generator.h>
#include <OpenPHRI/definitions.h>

#include <physical_quantities/spatial/impedance/mass.h>

namespace phri {

//! \brief Generates a force as if a virtual mass is attached to the robot.
class MassGenerator : public ForceGenerator {
public:
    /**
     * @brief Construct a mass generator given a mass and a target acceleration
     * in the TCP frame.
     * @param mass The virtual mass value.
     * @param target_acceleration The acceleration target in the TCP frame.
     * @param mass_frame The frame in which the mass is expressed.
     * @param target_acceleration_frame The frame in which the acceleration
     * target is expressed.
     */
    MassGenerator(std::shared_ptr<spatial::Mass> mass,
                  std::shared_ptr<spatial::Acceleration> target_acceleration);

    ~MassGenerator() = default;

protected:
    virtual void update(spatial::Force& force) override;

    std::shared_ptr<spatial::Mass> mass_;
    std::shared_ptr<spatial::Acceleration> target_acceleration_;
    // spatial::Frame mass_frame_;
};

using MassGeneratorPtr = std::shared_ptr<MassGenerator>;
using MassGeneratorConstPtr = std::shared_ptr<const MassGenerator>;

} // namespace phri
