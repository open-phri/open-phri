/*      File: safety_controller.cpp
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

#include <boost/python.hpp>
#include <iostream>

namespace phri {

std::shared_ptr<SafetyController>
NewSafetyController(std::shared_ptr<Matrix6d> damping_matrix) {
    return std::make_shared<SafetyController>(damping_matrix);
}

// Safety controller methods overloads
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SafetyController_addConstraint_overloads,
                                       addConstraint, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
    SafetyController_addVelocityGenerator_overloads, addVelocityGenerator, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
    SafetyController_addForceGenerator_overloads, addForceGenerator, 2, 3)

} // namespace phri

void wrapSafetyController() {
    using namespace phri;
    using namespace boost::python;

    /*********************************************************************************/
    /*                          SafetyController bindings */
    /*********************************************************************************/
    def("NewSafetyController", &NewSafetyController,
        "Create a new instance of a SafetyController shared_ptr");

    class_<SafetyController, boost::noncopyable>(
        "SafetyController",
        "Generates a tool velocity based on force and velocity inputs and a "
        "set of constraints",
        no_init)
        .def("setVerbose", &SafetyController::setVerbose)
        .def("addConstraint", &SafetyController::addConstraint,
             SafetyController_addConstraint_overloads(
                 args("name", "constraint", "force")))
        .def("addForceGenerator", &SafetyController::addForceGenerator,
             SafetyController_addForceGenerator_overloads(
                 args("name", "generator", "force")))
        .def("addVelocityGenerator", &SafetyController::addVelocityGenerator,
             SafetyController_addVelocityGenerator_overloads(
                 args("name", "generator", "force")))
        .def("removeConstraint", &SafetyController::removeConstraint)
        .def("removeForceGenerator", &SafetyController::removeForceGenerator)
        .def("removeVelocityGenerator",
             &SafetyController::removeVelocityGenerator)
        .def("getConstraint", &SafetyController::getConstraint)
        .def("getForceGenerator", &SafetyController::getForceGenerator)
        .def("getVelocityGenerator", &SafetyController::getVelocityGenerator)
        .def("updateTCPVelocity", &SafetyController::updateTCPVelocity)
        .def("getTCPVelocity", &SafetyController::getTCPVelocity)
        .def("getTotalVelocity", &SafetyController::getTotalVelocity)
        .def("getForceSum", &SafetyController::getForceSum)
        .def("getVelocitySum", &SafetyController::getVelocitySum);

    register_ptr_to_python<std::shared_ptr<SafetyController>>();
}
