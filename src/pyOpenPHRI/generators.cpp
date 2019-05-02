/*      File: generators.cpp
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

std::shared_ptr<VelocityProxy> NewVelocityProxy(Vector6dConstPtr velocity) {
    return std::make_shared<VelocityProxy>(velocity);
}

std::shared_ptr<ForceProxy> NewForceProxy(Vector6dConstPtr force) {
    return std::make_shared<ForceProxy>(force);
}

std::shared_ptr<PotentialFieldObject>
NewPotentialFieldObject(PotentialFieldType type, doubleConstPtr gain,
                        doubleConstPtr threshold_distance,
                        Vector6dConstPtr object_position) {
    return std::make_shared<PotentialFieldObject>(
        type, gain, threshold_distance, object_position);
}

std::shared_ptr<PotentialFieldGenerator>
NewPotentialFieldGenerator(Vector6dConstPtr robot_position) {
    return std::make_shared<PotentialFieldGenerator>(robot_position);
}

std::shared_ptr<PotentialFieldGenerator> NewPotentialFieldGenerator() {
    return std::make_shared<PotentialFieldGenerator>();
}
BOOST_PYTHON_FUNCTION_OVERLOADS(NewPotentialFieldGenerator_overloads,
                                NewPotentialFieldGenerator, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PotentialFieldGenerator_add_overloads,
                                       add, 2, 3)

std::shared_ptr<StiffnessGenerator>
NewStiffnessGenerator(Matrix6dConstPtr stiffness,
                      Vector6dConstPtr target_position) {
    return std::make_shared<StiffnessGenerator>(stiffness, target_position);
}

std::shared_ptr<StiffnessGenerator>
NewStiffnessGenerator(Matrix6dConstPtr stiffness,
                      Vector6dConstPtr target_position,
                      Vector6dConstPtr robot_position) {
    return std::make_shared<StiffnessGenerator>(stiffness, target_position,
                                                robot_position);
}
BOOST_PYTHON_FUNCTION_OVERLOADS(NewStiffnessGenerator_overloads,
                                NewStiffnessGenerator, 2, 3)

std::shared_ptr<MassGenerator>
NewMassGenerator(Matrix6dConstPtr mass, Vector6dConstPtr target_acceleration) {
    return std::make_shared<MassGenerator>(mass, target_acceleration);
}

std::shared_ptr<MassGenerator>
NewMassGenerator(Matrix6dConstPtr mass, Vector6dConstPtr target_acceleration,
                 Vector6dConstPtr robot_acceleration) {
    return std::make_shared<MassGenerator>(mass, target_acceleration,
                                           robot_acceleration);
}
BOOST_PYTHON_FUNCTION_OVERLOADS(NewMassGenerator_overloads, NewMassGenerator, 2,
                                3)

} // namespace phri

void wrapGenerators() {
    using namespace phri;
    using namespace boost::python;

    /**********************************************************************************/
    /*                                 Generators bindings */
    /**********************************************************************************/
    def("NewVelocityProxy", NewVelocityProxy,
        "Create a new instance of a VelocityProxy shared_ptr");
    def("NewForceProxy", NewForceProxy,
        "Create a new instance of a ForceProxy shared_ptr");
    def("NewPotentialFieldObject", NewPotentialFieldObject,
        "Create a new instance of a PotentialFieldObject shared_ptr");

    def("NewPotentialFieldGenerator",
        (std::shared_ptr<PotentialFieldGenerator>(*)(Vector6dPtr))0,
        NewPotentialFieldGenerator_overloads(
            args("rob_pos"),
            "Create a new instance of a PotentialFieldGenerator shared_ptr"));

    def("NewStiffnessGenerator",
        (std::shared_ptr<StiffnessGenerator>(*)(
            Matrix6dConstPtr, Vector6dConstPtr, Vector6dConstPtr))0,
        NewStiffnessGenerator_overloads(
            args("stiffness", "target_position", "robot_position"),
            "Create a new instance of a StiffnessGenerator shared_ptr"));

    def("NewMassGenerator",
        (std::shared_ptr<MassGenerator>(*)(Matrix6dConstPtr, Vector6dConstPtr,
                                           Vector6dConstPtr))0,
        NewMassGenerator_overloads(
            args("mass", "target_acceleration", "robot_acceleration"),
            "Create a new instance of a MassGenerator shared_ptr"));

    struct VelocityGeneratorWrap : VelocityGenerator,
                                   wrapper<VelocityGenerator> {
        using VelocityGenerator::VelocityGenerator;

        Vector6d compute() {
            return this->get_override("compute")();
        }
    };

    class_<VelocityGeneratorWrap, boost::noncopyable>("VelocityGenerator",
                                                      no_init)
        .def("compute", pure_virtual(&VelocityGenerator::compute));

    class_<VelocityProxy, boost::noncopyable, bases<VelocityGenerator>>(
        "VelocityProxy", no_init)
        .def("compute", &VelocityProxy::compute);

    struct ForceGeneratorWrap : ForceGenerator, wrapper<ForceGenerator> {
        using ForceGenerator::ForceGenerator;

        Vector6d compute() {
            return this->get_override("compute")();
        }
    };

    class_<ForceGeneratorWrap, boost::noncopyable>("ForceGenerator", no_init)
        .def("compute", pure_virtual(&ForceGenerator::compute));

    class_<ForceProxy, boost::noncopyable, bases<ForceGenerator>>("ForceProxy",
                                                                  no_init)
        .def("compute", &ForceProxy::compute);

    enum_<PotentialFieldType>("PotentialFieldType", "Type of potential field")
        .value("Attractive", PotentialFieldType::Attractive)
        .value("Repulsive", PotentialFieldType::Repulsive);

    class_<PotentialFieldObject>("PotentialFieldObject",
                                 init<PotentialFieldType, doubleConstPtr,
                                      doubleConstPtr, Vector6dConstPtr>());

    class_<ObjectCollection<PotentialFieldObjectPtr>>(
        "PotentialFieldObjectCollection", no_init);

    void (PotentialFieldGenerator::*PotentialFieldGenerator_setVerbose)(bool) =
        &PotentialFieldGenerator::setVerbose;
    class_<PotentialFieldGenerator, boost::noncopyable,
           bases<ForceGenerator, ObjectCollection<PotentialFieldObjectPtr>>>(
        "PotentialFieldGenerator", no_init)
        .def("setVerbose", PotentialFieldGenerator_setVerbose)
        .def("compute", &PotentialFieldGenerator::compute)
        .def("add", &PotentialFieldGenerator::add,
             PotentialFieldGenerator_add_overloads(
                 args("name", "object", "force")))
        .def("remove", &PotentialFieldGenerator::remove)
        .def("get", &PotentialFieldGenerator::get);

    class_<StiffnessGenerator, boost::noncopyable, bases<ForceGenerator>>(
        "StiffnessGenerator", no_init)
        .def("compute", &StiffnessGenerator::compute);

    class_<MassGenerator, boost::noncopyable, bases<ForceGenerator>>(
        "MassGenerator", no_init)
        .def("compute", &MassGenerator::compute);

    register_ptr_to_python<std::shared_ptr<VelocityProxy>>();
    register_ptr_to_python<std::shared_ptr<ForceProxy>>();
    register_ptr_to_python<std::shared_ptr<PotentialFieldObject>>();
    register_ptr_to_python<std::shared_ptr<PotentialFieldGenerator>>();
    register_ptr_to_python<std::shared_ptr<StiffnessGenerator>>();
    register_ptr_to_python<std::shared_ptr<MassGenerator>>();

    implicitly_convertible<std::shared_ptr<VelocityProxy>,
                           std::shared_ptr<VelocityGenerator>>();
    implicitly_convertible<std::shared_ptr<ForceProxy>,
                           std::shared_ptr<ForceGenerator>>();
    implicitly_convertible<std::shared_ptr<PotentialFieldGenerator>,
                           std::shared_ptr<ForceGenerator>>();
    implicitly_convertible<std::shared_ptr<StiffnessGenerator>,
                           std::shared_ptr<ForceGenerator>>();
    implicitly_convertible<std::shared_ptr<MassGenerator>,
                           std::shared_ptr<ForceGenerator>>();
}
