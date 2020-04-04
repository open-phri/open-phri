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
#include <OpenPHRI/detail/universal_wrapper.hpp>

#include <physical_quantities/spatial/impedance/stiffness.h>

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
    StiffnessGenerator();

    //! \brief Construct a new StiffnessGenerator object using the
    //! given spatial::Stiffness and spatial::Position values, references or
    //! (shared) pointers
    //!
    //! If either stiffness/target_pose are a const
    //! references/pointers, using stiffness()/targetPose()
    //! to modify them will result in undefined behavior
    //!
    //! \tparam StiffnessT The type of the value (automatically deduced)
    //! \tparam PoseT The type of the value (automatically deduced)
    //! \param stiffness The desired spatial stiffness
    //! \param target_pose The desired target pose
    template <typename StiffnessT, typename PoseT>
    explicit StiffnessGenerator(StiffnessT&& stiffness,
                                PoseT&& target_pose) noexcept
        : stiffness_{std::forward<StiffnessT>(stiffness)},
          target_pose_{std::forward<PoseT>(target_pose)} {
    }

    //! \brief Read/write access the stiffness used by the generator
    //! \return double& A reference to the stiffness
    void setStiffness(const spatial::Stiffness& stiffness);

    //! \brief Read access the stiffness used by the generator
    //! \return double The stiffness value
    const spatial::Stiffness& getStiffness() const;

    //! \brief Read/write access the target pose used by the generator
    //! \return double& A reference to the target pose
    void setTargetPose(const spatial::Position& position);

    //! \brief Read access the target pose used by the generator
    //! \return double The target pose value
    const spatial::Position& getTargetPose() const;

protected:
    void update(spatial::Force& force) override;

    detail::UniversalWrapper<spatial::Stiffness> stiffness_;
    detail::UniversalWrapper<spatial::Position> target_pose_;
    // spatial::Frame stiffness_frame_;
};

} // namespace phri
