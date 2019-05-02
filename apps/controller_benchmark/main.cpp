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

constexpr bool USE_LOOP = false;

int main(int argc, char const* argv[]) {

    auto robot = make_shared<Robot>("rob", // Robot's name
                                    7);    // Robot's joint count

    auto safety_controller = SafetyController(robot);
    safety_controller.skipJacobianInverseComputation(true);

    Clock clock;
    DataLogger logger("/mnt/tmpfs/open-phri_logs", clock.getTime(),
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
    auto vmax = make_shared<double>(0.1);
    safety_controller.add("vel cstr", VelocityConstraint(vmax));
    run_benchmark();

    // Velocity + power constraint
    logger.reset();
    logger.logExternalData("tavg_controller_2cstr", &t_avg, 1);
    auto pmax = make_shared<double>(0.1);
    safety_controller.add("pow cstr", PowerConstraint(pmax));
    run_benchmark();

    // Velocity + power + kinetic energy constraint
    logger.reset();
    logger.logExternalData("tavg_controller_3cstr", &t_avg, 1);
    auto emax = make_shared<double>(0.1);
    auto inertia = make_shared<MatrixXd>();
    inertia->resize(7, 7);
    inertia->setIdentity();
    ManipulatorEquivalentMass mass_eq(inertia, robot->jacobian());
    mass_eq.add("operator", std::make_shared<Vector6d>(Vector6d::Ones()));
    safety_controller.add(
        "ec cstr", KineticEnergyConstraint(emax, mass_eq.getEquivalentMass()));
    runner = [&safety_controller, &mass_eq]() {
        mass_eq.compute();
        safety_controller.compute();
    };
    run_benchmark();

    // Velocity + power + kinetic energy constraint + PFM + Stiffness + force
    // control
    logger.reset();
    logger.logExternalData("tavg_controller_3cstr_3gen", &t_avg, 1);
    auto potential_field_generator = make_shared<PotentialFieldGenerator>();
    auto obstacle = make_shared<PotentialFieldObject>(
        PotentialFieldType::Repulsive, make_shared<double>(10.),
        make_shared<double>(0.2),
        make_shared<Pose>(Vector3d::Ones(), Eigen::Quaterniond::Identity()));
    auto target = make_shared<PotentialFieldObject>(
        PotentialFieldType::Attractive, make_shared<double>(10.),
        make_shared<double>(std::numeric_limits<double>::infinity()),
        make_shared<Pose>(Vector3d::Ones() * 2.,
                          Eigen::Quaterniond::Identity()));
    potential_field_generator->add("obstacle", obstacle);
    potential_field_generator->add("target", target);
    safety_controller.add("pfm", potential_field_generator);
    safety_controller.add("stiffness",
                          std::make_shared<StiffnessGenerator>(
                              make_shared<Matrix6d>(Matrix6d::Identity()),
                              robot->controlPointTargetPose()));
    auto target_force = std::make_shared<Vector6d>(Vector6d::Zero());
    auto p_gain = std::make_shared<Vector6d>(Vector6d::Ones() * 0.005);
    auto d_gain = std::make_shared<Vector6d>(Vector6d::Ones() * 0.00005);
    auto selection = std::make_shared<Vector6d>(Vector6d::Zero());
    target_force->z() = 10.;
    selection->z() = 1.;
    safety_controller.add(
        "force control",
        ForceControl(target_force, 0.001, p_gain, d_gain, selection));
    run_benchmark();

    return 0;
}
