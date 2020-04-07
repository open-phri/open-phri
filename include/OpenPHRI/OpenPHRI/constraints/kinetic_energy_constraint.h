/*      File: kinetic_energy_constraint.h
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

//! \file kinetic_energy_constraint.h
//! \author Benjamin Navarro
//! \brief A constraint to limit the robot's kinetic energy
//! \date 05-2019
//! \ingroup phri

#pragma once

#include <OpenPHRI/constraints/velocity_constraint.h>

#include <physical_quantities/scalar/velocity.h>
#include <physical_quantities/scalar/mass.h>
#include <physical_quantities/scalar/energy.h>

namespace phri {

//! \brief A constraint to limit the robot's kinetic energy.
//! \details Works with a point mass model. For a manipulator, see
//! ManipulatorEquivalentMass to get its equivalent mass.
class KineticEnergyConstraint : public VelocityConstraint {
public:
    //! \brief Construct a new KineticEnergyConstraint object with an
    //! initial thresholds set to zero.
    //! \details Use KineticEnergyConstraint::mass() and
    //! KineticEnergyConstraint::maximumKinectEnergy() to set them to
    //! the desired value.
    KineticEnergyConstraint();

    //! \brief Construct a new KineticEnergyConstraint object using the
    //! given a spatial::Force and ForceControl::Parameters values, references
    //! or (shared) pointers
    //!
    //! If either target/parameters are a const references/pointers, using
    //! target()/parameters() to modify them will result in undefined behavior
    //!
    //! \tparam MassT The type of the value (automatically deduced)
    //! \tparam EnergyT The type of the value (automatically deduced)
    //! \param mass The mass or equivalent mass of the robot (kg)
    //! \param maximum_kinetic_energy A shared pointer to the maximum kinetic
    //! energy (J)
    template <typename MassT, typename EnergyT>
    explicit KineticEnergyConstraint(MassT&& mass,
                                     EnergyT&& maximum_kinetic_energy) noexcept
        : mass_{std::forward<MassT>(mass)},
          maximum_kinetic_energy_{
              std::forward<EnergyT>(maximum_kinetic_energy)} {
        setMaximumVelocity(scalar::Velocity{0.});
    }

    [[nodiscard]] double compute() override;

    //! \brief Read/write access the mass used by the constraint
    //! \return double& A reference to the mass
    void setMass(const scalar::Mass& mass);

    //! \brief Read access the mass used by the constraint
    //! \return double The mass value
    [[nodiscard]] const scalar::Mass& getMass() const;

    //! \brief Read/write access the kinetic energy limit used by the constraint
    //! \return double& A reference to the kinetic energy limit
    void setMaximumKineticEnergy(const scalar::Energy& energy);

    //! \brief Read access the kinetic energy limit used by the constraint
    //! \return double The kinetic energy limit value
    [[nodiscard]] const scalar::Energy& getMaximumKineticEnergy() const;

private:
    using VelocityConstraint::setMaximumVelocity;

    detail::UniversalWrapper<scalar::Mass> mass_;
    detail::UniversalWrapper<scalar::Energy> maximum_kinetic_energy_;
};

} // namespace phri
