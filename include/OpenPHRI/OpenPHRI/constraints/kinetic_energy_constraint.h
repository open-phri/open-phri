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
    //! given pointed values
    //! \param mass The mass or equivalent mass of the robot (kg). Throws if
    //! the pointer is empty.
    //! \param maximum_kinetic_energy A shared pointer to the maximum kinetic
    //! energy (J). Throws if the pointer is empty.
    KineticEnergyConstraint(std::shared_ptr<double> mass,
                            std::shared_ptr<double> maximum_kinetic_energy);

    //! \brief Construct a new KineticEnergyConstraint object using
    //! the given referenced values
    //! \param mass A reference to the mass or equivalent mass of the robot
    //! (kg). Make sure that \p mass outlives the constraint
    //! \param maximum_kinetic_energy A reference to the maximum kinetic
    //! energy (J). Make sure that \p maximum_kinetic_energy outlives
    //! the constraint
    KineticEnergyConstraint(double& mass, double& maximum_kinetic_energy);

    //! \brief Construct a new KineticEnergyConstraint object using the given
    //! values
    //! \param mass The value of the mass or equivalent mass of the robot (kg)
    //! \param maximum_kinetic_energy The value of the maximum kinetic energy
    //! (J)
    KineticEnergyConstraint(const double& mass,
                            const double& maximum_kinetic_energy);

    //! \brief Construct a new KineticEnergyConstraint object using
    //! the given values
    //! \param mass The value of the mass or equivalent mass of the robot (kg)
    //! \param maximum_kinetic_energy The value of the maximum kinetic energy
    //! (J)
    KineticEnergyConstraint(double&& mass, double&& maximum_kinetic_energy);

    //! \brief Default copy constructor
    KineticEnergyConstraint(const KineticEnergyConstraint&) = default;

    //! \brief Default move constructor
    KineticEnergyConstraint(KineticEnergyConstraint&&) = default;

    //! \brief Default virtual destructor
    virtual ~KineticEnergyConstraint() = default;

    //! \brief Default copy operator
    KineticEnergyConstraint&
    operator=(const KineticEnergyConstraint&) = default;

    //! \brief Default move operator
    KineticEnergyConstraint& operator=(KineticEnergyConstraint&&) = default;

    virtual double compute() override;

    //! \brief Read/write access the mass used by the constraint
    //! \return double& A reference to the mass
    double& mass();

    //! \brief Read access the mass used by the constraint
    //! \return double The mass value
    double mass() const;

    //! \brief Access to the shared pointer holding the mass
    //! used by the constraint
    //! \return std::shared_ptr<double> A shared pointer to the mass
    std::shared_ptr<double> massPtr();

    //! \brief Read/write access the kinetic energy limit used by the constraint
    //! \return double& A reference to the kinetic energy limit
    double& maximumKineticEnergy();

    //! \brief Read access the kinetic energy limit used by the constraint
    //! \return double The kinetic energy limit value
    double maximumKineticEnergy() const;

    //! \brief Access to the shared pointer holding the kinetic energy limit
    //! used by the constraint
    //! \return std::shared_ptr<double> A shared pointer to the kinetic energy
    //! limit
    std::shared_ptr<double> maximumKineticEnergyPtr();

private:
    void create();

    using VelocityConstraint::maximumVelocity;
    using VelocityConstraint::maximumVelocityPtr;

    std::shared_ptr<double> mass_;
    std::shared_ptr<double> maximum_kinetic_energy_;
    std::shared_ptr<double> kinetic_energy_maximum_velocity_;
};

} // namespace phri
