/*      File: utilities.cpp
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

template <typename T>
std::shared_ptr<Integrator<T>> NewIntegrator(std::shared_ptr<const T> input,
                                             double sample_time) {
    return std::make_shared<Integrator<T>>(input, sample_time);
}
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Integrator_force_overloads, force, 1,
// 1)

template <typename T>
std::shared_ptr<Derivator<T>> NewDerivator(std::shared_ptr<const T> input,
                                           double sample_time) {
    return std::make_shared<Derivator<T>>(input, sample_time);
}

} // namespace phri

#define CREATE_INTEGRATOR(type)                                                \
    void (Integrator<type>::*force_val_##type)(const type&) =                  \
        &Integrator<type>::force;                                              \
    void (Integrator<type>::*force_ptr_##type)(std::shared_ptr<const type>) =  \
        &Integrator<type>::force;                                              \
    class_<Integrator<type>, boost::noncopyable>("Integrator", no_init)        \
        .def("getOutput", &Integrator<type>::getOutput)                        \
        .def("reset", &Integrator<type>::reset)                                \
        .def("force", force_val_##type)                                        \
        .def("force", force_ptr_##type)                                        \
        .def("compute", &Integrator<type>::compute);

#define CREATE_DERIVATOR(type)                                                 \
    void (Derivator<type>::*reset##type)() = &Derivator<type>::reset;          \
    class_<Derivator<type>, boost::noncopyable>("Derivator", no_init)          \
        .def("getOutput", &Derivator<type>::getOutput)                         \
        .def("reset", reset##type)                                             \
        .def("compute", &Derivator<type>::compute);

#define CREATE_OBJECT_COLLECTION(type)                                         \
    void (ObjectCollection<type>::*setVerbose##type)(bool) =                   \
        &ObjectCollection<type>::setVerbose;                                   \
    class_<ObjectCollection<type>, boost::noncopyable>("ObjectCollection",     \
                                                       no_init)                \
        .def("reset", setVerbose##type)                                        \
        .def("add", &ObjectCollection<type>::add)                              \
        .def("remove", &ObjectCollection<type>::remove)                        \
        .def("get", &ObjectCollection<type>::get);

void wrapUtilities() {
    using namespace phri;
    using namespace boost::python;

    /*********************************************************************************/
    /*                            Interpolators bindings */
    /*********************************************************************************/
    def("NewIntegrator", NewIntegrator<double>,
        "Create a new instance of a Integrator shared_ptr");
    def("NewIntegrator", NewIntegrator<Vector2d>,
        "Create a new instance of a Integrator shared_ptr");
    def("NewIntegrator", NewIntegrator<Vector3d>,
        "Create a new instance of a Integrator shared_ptr");
    def("NewIntegrator", NewIntegrator<Vector6d>,
        "Create a new instance of a Integrator shared_ptr");

    def("NewDerivator", NewDerivator<double>,
        "Create a new instance of a Derivator shared_ptr");
    def("NewDerivator", NewDerivator<Vector2d>,
        "Create a new instance of a Derivator shared_ptr");
    def("NewDerivator", NewDerivator<Vector3d>,
        "Create a new instance of a Derivator shared_ptr");
    def("NewDerivator", NewDerivator<Vector6d>,
        "Create a new instance of a Derivator shared_ptr");

    CREATE_INTEGRATOR(double)
    CREATE_INTEGRATOR(Vector2d)
    CREATE_INTEGRATOR(Vector3d)
    CREATE_INTEGRATOR(Vector6d)

    CREATE_DERIVATOR(double)
    CREATE_DERIVATOR(Vector2d)
    CREATE_DERIVATOR(Vector3d)
    CREATE_DERIVATOR(Vector6d)

    CREATE_OBJECT_COLLECTION(Vector6dConstPtr)
    CREATE_OBJECT_COLLECTION(PotentialFieldObjectPtr)
    CREATE_OBJECT_COLLECTION(TrajectoryPtr)

    register_ptr_to_python<std::shared_ptr<Integrator<double>>>();
    register_ptr_to_python<std::shared_ptr<Integrator<Vector2d>>>();
    register_ptr_to_python<std::shared_ptr<Integrator<Vector3d>>>();
    register_ptr_to_python<std::shared_ptr<Integrator<Vector6d>>>();

    register_ptr_to_python<std::shared_ptr<Derivator<double>>>();
    register_ptr_to_python<std::shared_ptr<Derivator<Vector2d>>>();
    register_ptr_to_python<std::shared_ptr<Derivator<Vector3d>>>();
    register_ptr_to_python<std::shared_ptr<Derivator<Vector6d>>>();
}
