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
#include <OpenPHRI/detail/universal_wrapper.hpp>

#include <physical_quantities/spatial/impedance/mass.h>

namespace phri {

//! \brief Generates a force as if a virtual mass is attached to the robot.
class MassGenerator : public ForceGenerator {
public:
    //! \brief Construct a new MassGenerator object with an
    //! initial mass and target acceleration set to zero.
    //! \details Use MassGenerator::mass() and
    //! MassGenerator::targetAcceleration() to set them to
    //! the desired value
    MassGenerator();

    //! \brief Construct a new MassGenerator object using the
    //! given spatial::Mass and spatial::Acceleration values, references or
    //! (shared) pointers
    //!
    //! If either mass/target_acceleration are a const
    //! references/pointers, using mass()/targetAcceleration()
    //! to modify them will result in undefined behavior
    //!
    //! \tparam MassT The type of the value (automatically deduced)
    //! \tparam AccT The type of the value (automatically deduced)
    //! \param mass The desired spatial mass
    //! \param target_acceleration The desired target acceleration
    template <typename MassT, typename AccT>
    explicit MassGenerator(MassT&& mass, AccT&& target_acceleration) noexcept
        : mass_{std::forward<MassT>(mass)},
          target_acceleration_{std::forward<AccT>(target_acceleration)} {
    }

    //! \brief Read/write access the mass used by the generator
    //! \return double& A reference to the mass
    spatial::Mass& mass();

    //! \brief Read access the mass used by the generator
    //! \return double The mass value
    const spatial::Mass& mass() const;

    //! \brief Read/write access the targetAcceleration used by the generator
    //! \return double& A reference to the targetAcceleration
    spatial::Acceleration& targetAcceleration();

    //! \brief Read access the targetAcceleration used by the generator
    //! \return double The targetAcceleration value
    const spatial::Acceleration& targetAcceleration() const;

protected:
    virtual void update(spatial::Force& force) override;

    detail::UniversalWrapper<spatial::Mass> mass_;
    detail::UniversalWrapper<spatial::Acceleration> target_acceleration_;
    // spatial::Frame mass_frame_;
};

} // namespace phri
