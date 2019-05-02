/*      File: stiffness_generator.h
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
 * @file stiffness_generator.h
 * @author Benjamin Navarro
 * @brief Definition of the StiffnessGenerator class
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/force_generators/force_generator.h>
#include <OpenPHRI/definitions.h>

namespace phri {

/** @brief Generates a force as if a virtual spring is attached to the robot.
 */
class StiffnessGenerator : public ForceGenerator {
public:
    /**
     * @brief Construct a stiffness generator given a stiffness and a target
     * position.
     * @param stiffness The virtual stiffness value.
     * @param target_position The position target.
     * @param stiffness_frame The frame in which the stiffness is expressed.
     * @param target_position_frame The frame in which the position target is
     * expressed.
     */
    StiffnessGenerator(Matrix6dConstPtr stiffness, PoseConstPtr target_position,
                       ReferenceFrame stiffness_frame = ReferenceFrame::TCP);

    ~StiffnessGenerator() = default;

protected:
    virtual void update(Vector6d& force) override;

    Matrix6dConstPtr stiffness_;
    PoseConstPtr target_position_;
    ReferenceFrame stiffness_frame_;
};

using StiffnessGeneratorPtr = std::shared_ptr<StiffnessGenerator>;
using StiffnessGeneratorConstPtr = std::shared_ptr<const StiffnessGenerator>;

} // namespace phri
