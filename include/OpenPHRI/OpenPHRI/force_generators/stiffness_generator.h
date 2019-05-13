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

//! \file stiffness_generator.h
//! \author Benjamin Navarro
//! \brief Generates a force as if a virtual spring is attached to the robot.
//! \date 05-2019
//! \ingroup phri

#pragma once

#include <OpenPHRI/force_generators/force_generator.h>
#include <OpenPHRI/definitions.h>

namespace phri {

//! \brief Generates a force as if a virtual spring is attached to the robot.
class StiffnessGenerator : public ForceGenerator {
public:
    //! \brief Construct a new StiffnessGenerator object with initial
    //! stiffness and pose set to zero.
    //! \details Use StiffnessGenerator::stiffness() and
    //! StiffnessGenerator::pose() to set it to the desired value
    //! \param stiffness_frame The reference frame in which  the stiffness is
    //! expressed
    explicit StiffnessGenerator(ReferenceFrame stiffness_frame);

    //! \brief Construct a stiffness generator given a stiffness and a target
    //! pose.
    //! \param stiffness The virtual stiffness value.
    //! \param target_pose The pose target.
    //! \param stiffness_frame The frame in which the stiffness is expressed.
    StiffnessGenerator(std::shared_ptr<Vector6d> stiffness,
                       std::shared_ptr<Pose> target_pose,
                       ReferenceFrame stiffness_frame);

    //! \brief Construct a stiffness generator given a stiffness and a target
    //! pose.
    //! \param stiffness The virtual stiffness value. Make sure that \p
    //! stiffness outlives the generator
    //! \param target_pose The pose target. Make sure that \p target_pose
    //! outlives the generator
    //! \param stiffness_frame The reference frame in which  the stiffness is
    //! expressed
    StiffnessGenerator(Vector6d& stiffness, Pose& target_pose,
                       ReferenceFrame stiffness_frame);

    //! \brief Construct a stiffness generator given a stiffness and a target
    //! pose.
    //! \param stiffness The virtual stiffness value.
    //! \param target_pose The pose target.
    //! \param stiffness_frame The frame in which the stiffness is expressed.
    StiffnessGenerator(const Vector6d& stiffness, const Pose& target_pose,
                       ReferenceFrame stiffness_frame);

    //! \brief Construct a stiffness generator given a stiffness and a target
    //! pose.
    //! \param stiffness The virtual stiffness value.
    //! \param target_pose The pose target.
    //! \param stiffness_frame The frame in which the stiffness is expressed.
    StiffnessGenerator(Vector6d&& stiffness, Pose&& target_pose,
                       ReferenceFrame stiffness_frame);

    //! \brief Default copy constructor
    StiffnessGenerator(const StiffnessGenerator&) = default;

    //! \brief Default move constructor
    StiffnessGenerator(StiffnessGenerator&&) = default;

    //! \brief Default virtual destructor
    //! \details If \ref StiffnessGenerator::external_force_ was
    //! created using an rvalue reference, the pointed memory won't be released
    virtual ~StiffnessGenerator() = default;

    //! \brief Default copy operator
    StiffnessGenerator& operator=(const StiffnessGenerator&) = default;

    //! \brief Default move operator
    StiffnessGenerator& operator=(StiffnessGenerator&&) = default;

    //! \brief Read/write access the stiffness used by the generator
    //! \return double& A reference to the stiffness
    Vector6d& stiffness();

    //! \brief Read access the stiffness used by the generator
    //! \return double The stiffness value
    const Vector6d& stiffness() const;

    //! \brief Access to the shared pointer holding the stiffness used
    //! by the generator
    //! \return std::shared_ptr<double> A shared pointer to the stiffness
    std::shared_ptr<Vector6d> stiffnessPtr() const;

    //! \brief Read/write access the target pose used by the generator
    //! \return double& A reference to the target pose
    Pose& targetPose();

    //! \brief Read access the target pose used by the generator
    //! \return double The target pose value
    const Pose& targetPose() const;

    //! \brief Access to the shared pointer holding the target pose used
    //! by the generator
    //! \return std::shared_ptr<double> A shared pointer to the target pose
    std::shared_ptr<Pose> targetPosePtr() const;

protected:
    virtual void update(Wrench& force) override;

    std::shared_ptr<Vector6d> stiffness_;
    std::shared_ptr<Pose> target_pose_;
    ReferenceFrame stiffness_frame_;
};

} // namespace phri
