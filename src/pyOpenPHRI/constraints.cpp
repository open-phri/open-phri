/*      File: constraints.cpp
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

#include <OpenPHRI/OpenPHRI.h>
#include "double_wrappers.h"

#include <boost/python.hpp>
#include <iostream>

namespace phri {

// Wrappers for classes returning pointers to double
struct PowerConstraintWrap : public PowerConstraint {
    using PowerConstraint::PowerConstraint;

    ConstDoubleWrap getPowerPy() {
        return ConstDoubleWrap(getPower());
    }
};

// Functions to create shared pointers to constraints
std::shared_ptr<DefaultConstraint> NewDefaultConstraint(ConstraintType type) {
    return std::make_shared<DefaultConstraint>(type);
}

std::shared_ptr<VelocityConstraint>
NewVelocityConstraint(doubleConstPtr maximum_velocity) {
    return std::make_shared<VelocityConstraint>(maximum_velocity);
}

std::shared_ptr<KineticEnergyConstraint>
NewKineticEnergyConstraint(doubleConstPtr mass,
                           doubleConstPtr maximum_kinetic_energy) {
    return std::make_shared<KineticEnergyConstraint>(mass,
                                                     maximum_kinetic_energy);
}

std::shared_ptr<PowerConstraintWrap>
NewPowerConstraint(Vector6dConstPtr external_force,
                   doubleConstPtr maximum_power) {
    return std::make_shared<PowerConstraintWrap>(external_force, maximum_power);
}

std::shared_ptr<EmergencyStopConstraint>
NewEmergencyStopConstraint(Vector6dConstPtr external_force,
                           doubleConstPtr activation_force_threshold,
                           doubleConstPtr deactivation_force_threshold) {
    return std::make_shared<EmergencyStopConstraint>(
        external_force, activation_force_threshold,
        deactivation_force_threshold);
}

std::shared_ptr<SeparationDistanceConstraint>
NewSeparationDistanceConstraint(ConstraintPtr constraint,
                                InterpolatorPtr interpolator) {
    return std::make_shared<SeparationDistanceConstraint>(constraint,
                                                          interpolator);
}

std::shared_ptr<SeparationDistanceConstraint>
NewSeparationDistanceConstraint(ConstraintPtr constraint,
                                InterpolatorPtr interpolator,
                                Vector6dConstPtr robot_position) {
    return std::make_shared<SeparationDistanceConstraint>(
        constraint, interpolator, robot_position);
}
BOOST_PYTHON_FUNCTION_OVERLOADS(NewSeparationDistanceConstraint_overloads,
                                NewSeparationDistanceConstraint, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
    SeparationDistanceConstraint_add_overloads, add, 2, 3)

} // namespace phri

void wrapConstraints() {
    using namespace phri;
    using namespace boost::python;

    /**********************************************************************************/
    /*                                 Constraint bindings */
    /**********************************************************************************/
    def("NewDefaultConstraint", NewDefaultConstraint,
        "Create a new instance of a DefaultConstraint shared_ptr");
    def("NewVelocityConstraint", NewVelocityConstraint,
        "Create a new instance of a VelocityConstraint shared_ptr");
    def("NewKineticEnergyConstraint", NewKineticEnergyConstraint,
        "Create a new instance of a KineticEnergyConstraint shared_ptr");
    def("NewPowerConstraint", NewPowerConstraint,
        "Create a new instance of a PowerConstraint shared_ptr");
    def("NewEmergencyStopConstraint", NewEmergencyStopConstraint,
        "Create a new instance of a EmergencyStopConstraint shared_ptr");
    def("NewSeparationDistanceConstraint",
        (std::shared_ptr<SeparationDistanceConstraint>(*)(
            ConstraintPtr, InterpolatorPtr, Vector6dConstPtr))0,
        NewSeparationDistanceConstraint_overloads(
            args("constraint", "interpolator", "robot_position"),
            "Create a new instance of a SeparationDistanceConstraint "
            "shared_ptr"));

    enum_<ConstraintType>("ConstraintsType", "Type of constraint")
        .value("Multiplicative", ConstraintType::Multiplicative)
        .value("Minimum", ConstraintType::Minimum);

    struct ConstraintWrap : Constraint, wrapper<Constraint> {
        using Constraint::Constraint;

        double compute() {
            return this->get_override("compute")();
        }
    };

    class_<ConstraintWrap, boost::noncopyable>("Constraint", no_init)
        .def("getType", &Constraint::getType)
        .def("compute", pure_virtual(&Constraint::compute));

    class_<DefaultConstraint, boost::noncopyable, bases<Constraint>>(
        "DefaultConstraint", "Default constraint, always evaluates to 1",
        no_init)
        .def("compute", &DefaultConstraint::compute);

    class_<VelocityConstraint, boost::noncopyable, bases<Constraint>>(
        "VelocityConstraint",
        "Velocity constraint, limits the tool's maximum velocity", no_init)
        .def("compute", &VelocityConstraint::compute);

    class_<KineticEnergyConstraint, boost::noncopyable, bases<Constraint>>(
        "KineticEnergyConstraint",
        "kinetic energy constraint, limits the tool's maximum kinetic energy",
        no_init)
        .def("compute", &KineticEnergyConstraint::compute);

    class_<PowerConstraintWrap, boost::noncopyable, bases<Constraint>>(
        "PowerConstraint",
        "Power constraint, limits the exchanged power between the tool and the "
        "environment",
        no_init)
        .def("compute", &PowerConstraintWrap::compute)
        .def("getPower", &PowerConstraintWrap::getPowerPy);

    class_<EmergencyStopConstraint, boost::noncopyable, bases<Constraint>>(
        "EmergencyStopConstraint",
        "Stop constraint, stops the tool when the external force is above a "
        "given limit",
        no_init)
        .def("compute", &EmergencyStopConstraint::compute);

    void (
        SeparationDistanceConstraint::*SeparationDistanceConstraint_setVerbose)(
        bool) = &SeparationDistanceConstraint::setVerbose;
    class_<SeparationDistanceConstraint, boost::noncopyable,
           bases<Constraint, ObjectCollection<Vector6dConstPtr>>>(
        "SeparationDistanceConstraint",
        "Separation distance constraint, adapts the constraint depending on "
        "the distance to the closest object",
        no_init)
        .def("compute", &SeparationDistanceConstraint::compute)
        .def("setVerbose", SeparationDistanceConstraint_setVerbose)
        .def("add", &SeparationDistanceConstraint::add,
             SeparationDistanceConstraint_add_overloads(
                 args("name", "object", "force")))
        .def("remove", &SeparationDistanceConstraint::remove)
        .def("get", &SeparationDistanceConstraint::get);

    register_ptr_to_python<std::shared_ptr<DefaultConstraint>>();
    register_ptr_to_python<std::shared_ptr<VelocityConstraint>>();
    register_ptr_to_python<std::shared_ptr<KineticEnergyConstraint>>();
    register_ptr_to_python<std::shared_ptr<PowerConstraintWrap>>();
    register_ptr_to_python<std::shared_ptr<EmergencyStopConstraint>>();
    register_ptr_to_python<std::shared_ptr<SeparationDistanceConstraint>>();

    implicitly_convertible<std::shared_ptr<DefaultConstraint>,
                           std::shared_ptr<Constraint>>();
    implicitly_convertible<std::shared_ptr<VelocityConstraint>,
                           std::shared_ptr<Constraint>>();
    implicitly_convertible<std::shared_ptr<KineticEnergyConstraint>,
                           std::shared_ptr<Constraint>>();
    implicitly_convertible<std::shared_ptr<PowerConstraintWrap>,
                           std::shared_ptr<Constraint>>();
    implicitly_convertible<std::shared_ptr<EmergencyStopConstraint>,
                           std::shared_ptr<Constraint>>();
    implicitly_convertible<std::shared_ptr<SeparationDistanceConstraint>,
                           std::shared_ptr<Constraint>>();
}
