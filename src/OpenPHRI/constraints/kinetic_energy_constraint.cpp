/*      File: kinetic_energy_constraint.cpp
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

#include <OpenPHRI/constraints/kinetic_energy_constraint.h>
#include <OpenPHRI/utilities/exceptions.h>

namespace phri {

KineticEnergyConstraint::KineticEnergyConstraint()
    : mass_(std::make_shared<double>(0.)),
      maximum_kinetic_energy_(std::make_shared<double>(0.)) {
    create();
}

KineticEnergyConstraint::KineticEnergyConstraint(
    std::shared_ptr<double> mass,
    std::shared_ptr<double> maximum_kinetic_energy)
    : mass_(mass), maximum_kinetic_energy_(maximum_kinetic_energy) {
    if (not mass or not maximum_kinetic_energy) {
        throw std::runtime_error(
            OPEN_PHRI_ERROR("You provided an empty shared pointer"));
    }
    create();
}

KineticEnergyConstraint::KineticEnergyConstraint(double& mass,
                                                 double& maximum_kinetic_energy)
    : KineticEnergyConstraint(
          std::shared_ptr<double>(&mass, [](auto p) {}),
          std::shared_ptr<double>(&maximum_kinetic_energy, [](auto p) {})) {
    create();
}

KineticEnergyConstraint::KineticEnergyConstraint(
    const double& mass, const double& maximum_kinetic_energy)
    : KineticEnergyConstraint(
          std::make_shared<double>(mass),
          std::make_shared<double>(maximum_kinetic_energy)) {
    create();
}

KineticEnergyConstraint::KineticEnergyConstraint(
    double&& mass, double&& maximum_kinetic_energy)
    : KineticEnergyConstraint(
          std::make_shared<double>(std::move(mass)),
          std::make_shared<double>(std::move(maximum_kinetic_energy))) {
    create();
}

double KineticEnergyConstraint::compute() {
    *kinetic_energy_maximum_velocity_ =
        std::sqrt(2. * *maximum_kinetic_energy_ / *mass_);

    return VelocityConstraint::compute();
}

void KineticEnergyConstraint::create() {
    kinetic_energy_maximum_velocity_ = std::make_shared<double>(0.);
    maximum_velocity_ = kinetic_energy_maximum_velocity_;
}

double& KineticEnergyConstraint::mass() {
    return *mass_;
}

double KineticEnergyConstraint::mass() const {
    return *mass_;
}

std::shared_ptr<double> KineticEnergyConstraint::massPtr() const {
    return mass_;
}

double& KineticEnergyConstraint::maximumKineticEnergy() {
    return *maximum_kinetic_energy_;
}

double KineticEnergyConstraint::maximumKineticEnergy() const {
    return *maximum_kinetic_energy_;
}

std::shared_ptr<double>
KineticEnergyConstraint::maximumKineticEnergyPtr() const {
    return maximum_kinetic_energy_;
}

} // namespace phri
