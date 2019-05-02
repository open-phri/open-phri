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

/**
 * @file mass_generator.h
 * @author Benjamin Navarro
 * @brief Definition of the MassGenerator class
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/force_generators/force_generator.h>
#include <OpenPHRI/definitions.h>

namespace phri {

/** @brief Generates a force as if a virtual mass is attached to the robot.
 */
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
    MassGenerator(
        Matrix6dConstPtr mass, AccelerationConstPtr target_acceleration,
        ReferenceFrame mass_frame = ReferenceFrame::TCP,
        ReferenceFrame target_acceleration_frame = ReferenceFrame::TCP);

    ~MassGenerator() = default;

protected:
    virtual void update(Vector6d& force) override;

    Matrix6dConstPtr mass_;
    AccelerationConstPtr target_acceleration_;
    ReferenceFrame mass_frame_;
    ReferenceFrame target_acceleration_frame_;
};

using MassGeneratorPtr = std::shared_ptr<MassGenerator>;
using MassGeneratorConstPtr = std::shared_ptr<const MassGenerator>;

} // namespace phri
