/*      File: null_space_motion.h
 *       This file is part of the program open-phri
 *       Program description : OpenPHRI: a generic framework to easily and
 * safely control robots in interactions with humans Copyright (C) 2018 -
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
 * @file null_space_motion.h
 * @author Benjamin Navarro
 * @brief Definition of the NullSpaceMotion class
 * @date April 2018
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/joint_velocity_generators/joint_velocity_generator.h>
#include <OpenPHRI/definitions.h>

namespace phri {

/** @brief Adds a joint velocity in the null space of the task jacobian.
 *  @details Can be useful to perform a secondary task such as joint limits
 * avoidance.
 */
class NullSpaceMotion : public JointVelocityGenerator {
public:
    explicit NullSpaceMotion(VectorXdConstPtr joint_velocity);
    virtual ~NullSpaceMotion() = default;

protected:
    virtual void update(VectorXd& velocity) override;

    VectorXdConstPtr joint_velocity_;
    MatrixXd null_space_projector_;
    MatrixXd identity_;
};

using NullSpaceMotionPtr = std::shared_ptr<NullSpaceMotion>;
using NullSpaceMotionConstPtr = std::shared_ptr<const NullSpaceMotion>;

} // namespace phri
