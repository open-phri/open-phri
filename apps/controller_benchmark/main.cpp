/*      File: main.cpp
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

using namespace phri;
using namespace std;

constexpr bool USE_LOOP = true;

int main(int argc, char const* argv[]) {
    using namespace spatial::literals;

    auto robot = Robot{"tcp"_frame, "base"_frame, "test_rob", 7};
    auto safety_controller = SafetyController{robot};
    safety_controller.skipJacobianInverseComputation(true);

    Clock clock;
    DataLogger logger("/tmp", clock.getTime(),
                      true,  // create gnuplot files
                      true); // delay disk write

    double t_start, t_end, t_avg = 0.;
    constexpr int NCalls = 10000;
    constexpr int Tries = 1000;
    auto getAvgTime = [&t_start, &t_end, NCalls]() {
        return (t_end - t_start) / double(NCalls);
    };
    std::function<void(void)> runner = [&safety_controller]() {
        safety_controller.compute();
    };

    auto run_benchmark = [&]() {
        std::cout << "Benchmark started" << std::endl;
        for (size_t i = 0; i < Tries; ++i) {
            if (USE_LOOP) {
                for (size_t j = 0; j < NCalls; ++j) {
                    t_start = clock();
                    runner();
                    t_end = clock();
                    t_avg += t_end - t_start;
                }
                t_avg /= double(NCalls);
            } else {
                t_start = clock();
                run_function<NCalls>(runner);
                t_end = clock();
                t_avg = getAvgTime();
            }
            t_avg *= 1e6;
            // std::cout << "Average time (us): " << t_avg << std::endl;
            logger();
        }
        std::cout << "Benchmark ended" << std::endl;
    };

    // Warm up
    std::cout << "Warming up" << std::endl;
    clock.reset();
    while (clock() < 3.)
        continue;

    logger.logExternalData("tavg_controller_only", &t_avg, 1);
    run_benchmark();

    // Velocity constraint
    logger.reset();
    logger.logExternalData("tavg_controller_1cstr", &t_avg, 1);
    auto vmax = scalar::Velocity{0.1};
    safety_controller.add("vel cstr", VelocityConstraint(vmax));
    run_benchmark();

    // Velocity + power constraint
    logger.reset();
    logger.logExternalData("tavg_controller_2cstr", &t_avg, 1);
    auto pmax = scalar::Power{0.1};
    safety_controller.add("pow cstr", PowerConstraint(pmax));
    run_benchmark();

    // Velocity + power + kinetic energy constraint
    logger.reset();
    logger.logExternalData("tavg_controller_3cstr", &t_avg, 1);
    auto emax = scalar::Energy{0.1};
    auto inertia = spatial::Mass::Identity(robot.controlPointFrame());
    ManipulatorEquivalentMass mass_eq(robot, inertia);
    mass_eq.add("operator", spatial::Position::Ones(robot.controlPointFrame()));
    safety_controller.add<KineticEnergyConstraint>(
        "ec cstr", mass_eq.getEquivalentMass(), emax);
    runner = [&safety_controller, &mass_eq]() {
        mass_eq.compute();
        safety_controller.compute();
    };
    run_benchmark();

    // Velocity + power + kinetic energy constraint + PFM + Stiffness + force
    // control
    logger.reset();
    logger.logExternalData("tavg_controller_3cstr_3gen", &t_avg, 1);
    auto potential_field_generator =
        std::make_shared<PotentialFieldGenerator>();
    auto obstacle = PotentialFieldObject{
        PotentialFieldType::Repulsive, 10., 0.2,
        spatial::Position{
            spatial::LinearPosition::Ones(robot.controlPointFrame()),
            spatial::AngularPosition::Zero(robot.controlPointFrame())}};

    auto target = PotentialFieldObject{
        PotentialFieldType::Attractive, 10.,
        std::numeric_limits<double>::infinity(),
        spatial::Position{
            spatial::LinearPosition::Constant(2., robot.controlPointFrame()),
            spatial::AngularPosition::Zero(robot.controlPointFrame())}};

    potential_field_generator->add("obstacle", obstacle);
    potential_field_generator->add("target", target);
    safety_controller.add("pfm", potential_field_generator);

    safety_controller.add<StiffnessGenerator>(
        "stiffness", spatial::Stiffness::Identity(robot.controlPointFrame()),
        spatial::Position::Zero(robot.controlPointFrame()));

    auto target_force = spatial::Force::Zero(robot.controlPointFrame());
    Eigen::Vector6d p_gain = Eigen::Vector6d::Ones() * 0.005;
    Eigen::Vector6d d_gain = Eigen::Vector6d::Ones() * 0.00005;
    std::array<bool, 6> selection;
    selection.fill(false);

    target_force.z() = 10.;
    selection[2] = true;
    safety_controller.add<ForceControl>(
        "force control", target_force,
        ForceControl::Parameters{p_gain, d_gain, selection},
        ForceControl::TargetType::Environment);
    run_benchmark();

    return 0;
}
