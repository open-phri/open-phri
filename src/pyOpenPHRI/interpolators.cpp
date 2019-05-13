/*      File: interpolators.cpp
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

struct TrajectoryWrap : public Trajectory {
    using Trajectory::Trajectory;

    ConstDoubleWrap getOutputPy() {
        return ConstDoubleWrap(getOutput());
    }
};

std::shared_ptr<TrajectoryPoint>
NewTrajectoryPointPtr(std::shared_ptr<double> y, std::shared_ptr<double> dy, std::shared_ptr<double> d2y) {
    return std::make_shared<TrajectoryPoint>(y, dy, d2y);
}

std::shared_ptr<TrajectoryPoint> NewTrajectoryPointVal(double y, double dy,
                                                       double d2y) {
    return std::make_shared<TrajectoryPoint>(y, dy, d2y);
}

std::shared_ptr<TrajectoryWrap> NewTrajectory(TrajectoryOutputType output_type,
                                              TrajectoryPointPtr start,
                                              double sample_time) {
    return std::make_shared<TrajectoryWrap>(output_type, start, sample_time);
}

std::shared_ptr<TrajectoryGenerator>
NewTrajectoryGenerator(TrajectorySynchronization sync) {
    return std::make_shared<TrajectoryGenerator>(sync);
}
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(TrajectoryGenerator_add_overloads, add,
                                       2, 3)

struct LinearInterpolatorWrap : public LinearInterpolator {
    using LinearInterpolator::LinearInterpolator;

    ConstDoubleWrap getOutputPy() {
        return ConstDoubleWrap(getOutput());
    }
};

struct PolynomialInterpolatorWrap : public PolynomialInterpolator {
    using PolynomialInterpolator::PolynomialInterpolator;

    ConstDoubleWrap getOutputPy() {
        return ConstDoubleWrap(getOutput());
    }
};

std::shared_ptr<LinearInterpolatorWrap>
NewLinearInterpolator(LinearPointPtr from, LinearPointPtr to) {
    return std::make_shared<LinearInterpolatorWrap>(from, to);
}

std::shared_ptr<LinearInterpolatorWrap>
NewLinearInterpolator(LinearPointPtr from, LinearPointPtr to, std::shared_ptr<double> input) {
    return std::make_shared<LinearInterpolatorWrap>(from, to, input);
}
BOOST_PYTHON_FUNCTION_OVERLOADS(NewLinearInterpolator_overloads,
                                NewLinearInterpolator, 2, 3)

std::shared_ptr<PolynomialInterpolatorWrap>
NewPolynomialInterpolator(PolynomialPointPtr from, PolynomialPointPtr to) {
    return std::make_shared<PolynomialInterpolatorWrap>(from, to);
}

std::shared_ptr<PolynomialInterpolatorWrap>
NewPolynomialInterpolator(PolynomialPointPtr from, PolynomialPointPtr to,
                          std::shared_ptr<double> input) {
    return std::make_shared<PolynomialInterpolatorWrap>(from, to, input);
}
BOOST_PYTHON_FUNCTION_OVERLOADS(NewPolynomialInterpolator_overloads,
                                NewPolynomialInterpolator, 2, 3)

} // namespace phri

void wrapInterpolators() {
    using namespace phri;
    using namespace boost::python;

    /*********************************************************************************/
    /*                         Trajectory generator bindings */
    /*********************************************************************************/
    def("NewTrajectoryPoint", NewTrajectoryPointPtr,
        "Create a new instance of a TrajectoryPoint shared_ptr");
    def("NewTrajectoryPoint", NewTrajectoryPointVal,
        "Create a new instance of a TrajectoryPoint shared_ptr");
    def("NewTrajectory", NewTrajectory,
        "Create a new instance of a Trajectory shared_ptr");
    def("NewTrajectoryGenerator", NewTrajectoryGenerator,
        "Create a new instance of a TrajectoryGenerator shared_ptr");

    class_<TrajectoryPoint>("TrajectoryPoint",
                            "Point used by the TrajectoryGenerator",
                            init<std::shared_ptr<double>, std::shared_ptr<double>, std::shared_ptr<double>>())
        .def(init<double, double, double>())
        .add_property(
            "y", +[](const TrajectoryPoint& pt) -> double { return *pt.y; },
            +[](TrajectoryPoint& pt, double v) { *pt.y = v; })
        .add_property(
            "dy", +[](const TrajectoryPoint& pt) -> double { return *pt.dy; },
            +[](TrajectoryPoint& pt, double v) { *pt.dy = v; })
        .add_property(
            "d2y", +[](const TrajectoryPoint& pt) -> double { return *pt.d2y; },
            +[](TrajectoryPoint& pt, double v) { *pt.d2y = v; });

    void (TrajectoryWrap::*addPathTo_Max)(TrajectoryPointPtr, double, double) =
        &TrajectoryWrap::addPathTo;
    void (TrajectoryWrap::*addPathTo_Fixed)(TrajectoryPointPtr, double) =
        &TrajectoryWrap::addPathTo;
    class_<TrajectoryWrap>(
        "Trajectory",
        "A Trajectory is composed of multiple TrajectoryPoint with time or "
        "velocity/acceleration constraints",
        init<TrajectoryOutputType, TrajectoryPointPtr, std::shared_ptr<double>, double>())
        .def(init<TrajectoryOutputType, TrajectoryPointPtr, double>())
        .def("addPathTo", addPathTo_Max)
        .def("addPathTo", addPathTo_Fixed)
        .def("getOutput", &TrajectoryWrap::getOutputPy)
        .def("computeParameters", &TrajectoryWrap::computeParameters)
        .def("computeTimings", &TrajectoryWrap::computeTimings)
        .def("compute", &TrajectoryWrap::compute)
        .def("getCurrentPathMinimumTime",
             &TrajectoryWrap::getCurrentPathMinimumTime)
        .def("getTrajectoryMinimumTime",
             &TrajectoryWrap::getTrajectoryMinimumTime)
        .def("getTrajectoryDuration", &TrajectoryWrap::getTrajectoryDuration)
        .def("getPathMinimumTime", &TrajectoryWrap::getPathMinimumTime)
        .def("getPathDuration", &TrajectoryWrap::getPathDuration)
        .def("getSegmentCount", &TrajectoryWrap::getSegmentCount)
        .def("setPaddingTime", &TrajectoryWrap::setPaddingTime)
        .def("setPathCurrentTime", &TrajectoryWrap::setPathCurrentTime);

    void (TrajectoryGenerator::*TrajectoryGenerator_setVerbose)(bool) =
        &TrajectoryGenerator::setVerbose;
    class_<TrajectoryGenerator, bases<ObjectCollection<TrajectoryPtr>>>(
        "TrajectoryGenerator",
        "A TrajectoryGenerator groups multiple Trajectory and offer "
        "synchronization methods",
        init<TrajectorySynchronization>())
        .def("setVerbose", TrajectoryGenerator_setVerbose)
        .def("setSynchronizationMethod",
             &TrajectoryGenerator::setSynchronizationMethod)
        .def("computeParameters", &TrajectoryGenerator::computeParameters)
        .def("compute", &TrajectoryGenerator::compute)
        .def("add", &TrajectoryGenerator::add,
             TrajectoryGenerator_add_overloads(args("name", "object", "force")))
        .def("remove", &TrajectoryGenerator::remove)
        .def("get", &TrajectoryGenerator::get);

    enum_<TrajectoryOutputType>("TrajectoryOutputType",
                                "Output type for a Trajectory")
        .value("Position", TrajectoryOutputType::Position)
        .value("Velocity", TrajectoryOutputType::Velocity)
        .value("Acceleration", TrajectoryOutputType::Acceleration);

    enum_<TrajectorySynchronization>(
        "TrajectorySynchronization",
        "Synchronization type for a TrajectoryGenerator")
        .value("NoSynchronization",
               TrajectorySynchronization::NoSynchronization)
        .value("SynchronizeWaypoints",
               TrajectorySynchronization::SynchronizeWaypoints)
        .value("SynchronizeTrajectory",
               TrajectorySynchronization::SynchronizeTrajectory);

    register_ptr_to_python<std::shared_ptr<TrajectoryPoint>>();
    register_ptr_to_python<std::shared_ptr<TrajectoryWrap>>();
    register_ptr_to_python<std::shared_ptr<TrajectoryGenerator>>();

    implicitly_convertible<std::shared_ptr<TrajectoryWrap>,
                           std::shared_ptr<Trajectory>>();

    /*********************************************************************************/
    /*                            Interpolators bindings */
    /*********************************************************************************/
    def("NewLinearInterpolator",
        (std::shared_ptr<LinearInterpolatorWrap>(*)(
            LinearPointPtr, LinearPointPtr, std::shared_ptr<double>))0,
        NewLinearInterpolator_overloads(
            args("from", "to", "rob_pos"),
            "Create a new instance of a LinearInterpolator shared_ptr"));

    def("NewPolynomialInterpolator",
        (std::shared_ptr<PolynomialInterpolatorWrap>(*)(
            PolynomialPointPtr, PolynomialPointPtr, std::shared_ptr<double>))0,
        NewPolynomialInterpolator_overloads(
            args("from", "to", "rob_pos"),
            "Create a new instance of a PolynomialInterpolator shared_ptr"));

    struct InterpolatorWrap : Interpolator,
                              boost::python::wrapper<Interpolator> {
        using Interpolator::Interpolator;

        void computeParameters() {
            this->get_override("computeParameters")();
        }

        double compute() {
            return this->get_override("compute")();
        }
    };

    class_<InterpolatorWrap, boost::noncopyable>("Interpolator", no_init)
        .def("setInput", &InterpolatorWrap::setInput)
        .def("computeParameters",
             pure_virtual(&InterpolatorWrap::computeParameters))
        .def("compute", pure_virtual(&InterpolatorWrap::compute));

    class_<LinearPoint>("LinearPoint",
                        "Interpolation point to use with LinearInterpolator",
                        init<std::shared_ptr<double>, std::shared_ptr<double>>())
        .def(init<double, double>())
        .add_property(
            "x", +[](const LinearPoint& pt) -> double { return *pt.x; },
            +[](LinearPoint& pt, double v) { *pt.x = v; })
        .add_property(
            "y", +[](const LinearPoint& pt) -> double { return *pt.y; },
            +[](LinearPoint& pt, double v) { *pt.y = v; });

    def(
        "NewLinearPoint",
        +[](std::shared_ptr<double> x, std::shared_ptr<double> y) -> std::shared_ptr<LinearPoint> {
            return std::make_shared<LinearPoint>(x, y);
        });
    def(
        "NewLinearPoint",
        +[](double x, double y) -> std::shared_ptr<LinearPoint> {
            return std::make_shared<LinearPoint>(x, y);
        });

    class_<LinearInterpolatorWrap, boost::noncopyable, bases<InterpolatorWrap>>(
        "LinearInterpolator", "Linear interpolator", no_init)
        .def("getOutput", &LinearInterpolatorWrap::getOutputPy)
        .def("compute", &LinearInterpolatorWrap::compute)
        .def("computeParameters", &LinearInterpolatorWrap::computeParameters)
        .def("enableSaturation", &LinearInterpolatorWrap::enableSaturation);

    class_<PolynomialPoint, bases<TrajectoryPoint>>(
        "PolynomialPoint",
        "Interpolation point to use with PolynomialInterpolator",
        init<std::shared_ptr<double>, std::shared_ptr<double>, std::shared_ptr<double>, std::shared_ptr<double>>())
        .def(init<double, double, double, double>())
        .add_property(
            "x", +[](const PolynomialPoint& pt) -> double { return *pt.x; },
            +[](PolynomialPoint& pt, double v) { *pt.x = v; });

    def(
        "NewPolynomialPoint",
        +[](std::shared_ptr<double> x, std::shared_ptr<double> y, std::shared_ptr<double> dy,
            std::shared_ptr<double> d2y) -> std::shared_ptr<PolynomialPoint> {
            return std::make_shared<PolynomialPoint>(x, y, dy, d2y);
        });
    def(
        "NewPolynomialPoint",
        +[](double x, double y, double dy,
            double d2y) -> std::shared_ptr<PolynomialPoint> {
            return std::make_shared<PolynomialPoint>(x, y, dy, d2y);
        });

    class_<PolynomialInterpolatorWrap, boost::noncopyable, bases<Interpolator>>(
        "PolynomialInterpolator", "5th order polynomial interpolator", no_init)
        .def("getOutput", &PolynomialInterpolatorWrap::getOutputPy)
        .def("compute", &PolynomialInterpolatorWrap::compute)
        .def("computeParameters",
             &PolynomialInterpolatorWrap::computeParameters);

    register_ptr_to_python<std::shared_ptr<LinearPoint>>();
    register_ptr_to_python<std::shared_ptr<LinearInterpolatorWrap>>();
    register_ptr_to_python<std::shared_ptr<PolynomialPoint>>();
    register_ptr_to_python<std::shared_ptr<PolynomialInterpolatorWrap>>();

    implicitly_convertible<std::shared_ptr<LinearInterpolatorWrap>,
                           std::shared_ptr<Interpolator>>();
    implicitly_convertible<std::shared_ptr<PolynomialInterpolatorWrap>,
                           std::shared_ptr<Interpolator>>();
}
