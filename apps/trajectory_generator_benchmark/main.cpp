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

#include <iostream>
#include <type_traits>

#include <OpenPHRI/OpenPHRI.h>

using namespace std;
using namespace phri;

constexpr double SAMPLE_TIME = 0.010;
constexpr bool USE_LOOP = false;

int main(int argc, char const* argv[]) {
    auto point_1 = TrajectoryPoint<Vector3d>(
        Vector3d(0., 0., 0.), Vector3d(0., 0., 0.1), Vector3d(0., 0., 0.1));
    auto point_2 = TrajectoryPoint<Vector3d>(Vector3d(0.1, 0.2, 0.2),
                                             Vector3d(0., 0.1, 0.05),
                                             Vector3d(0., 0.05, -0.05));
    auto point_3 = TrajectoryPoint<Vector3d>(
        Vector3d(0.3, 0.3, 0.1), Vector3d(0., 0., -0.1), Vector3d(0., 0., 0.1));

    auto trajectory_generator =
        TrajectoryGenerator<Vector3d>(point_1, SAMPLE_TIME);

    trajectory_generator.addPathTo(point_2, Vector3d(0.05, 0.15, 0.2),
                                   Vector3d(0.01, 0.1, 0.2));
    trajectory_generator.addPathTo(point_3, Vector3d(0.1, 0.25, 0.2),
                                   Vector3d(0.02, 0.2, 0.2));
    trajectory_generator.addPathTo(point_1, Vector3d(0.05, 0.15, 0.2),
                                   Vector3d(0.01, 0.1, 0.2));
    trajectory_generator.addPathTo(point_3, Vector3d(0.1, 0.25, 0.2),
                                   Vector3d(0.02, 0.2, 0.2));
    trajectory_generator.addPathTo(point_2, Vector3d(0.05, 0.15, 0.2),
                                   Vector3d(0.01, 0.1, 0.2));
    trajectory_generator.addPathTo(point_1, Vector3d(0.05, 0.25, 0.2),
                                   Vector3d(0.01, 0.2, 0.2));

    // constexpr size_t N = 100000;
    Clock clock;
    DataLogger logger("/tmp/open-phri_logs", clock.getTime(),
                      true,  // create gnuplot files
                      true); // delay disk write

    double t_start, t_end, t_avg = 0., iter_avg = 0.;
    constexpr int NCalls = 2000;
    constexpr int Tries = 100;
    auto getAvgTime = [&t_start, &t_end, NCalls]() {
        return (t_end - t_start) / double(NCalls);
    };
    std::function<void(void)> runner = [&trajectory_generator]() {
        trajectory_generator.computeTimings(1e-6, 1e-6);
    };

    auto run_benchmark = [&]() {
        std::cout << "Benchmark started" << std::endl;
        for (size_t i = 0; i < Tries; ++i) {
            TrajectoryGenerator<Vector3d>::resetComputeTimingsIterations();
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
            iter_avg = double(TrajectoryGenerator<
                              Vector3d>::getComputeTimingsIterations()) /
                       double(NCalls);
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

#if 1
    map<string, std::function<void(void)>> epsilons;
    epsilons["_eps_1e-6"] = [&trajectory_generator]() {
        trajectory_generator.computeTimings(1e-6, 1e-6);
    };
    epsilons["_eps_1e-3"] = [&trajectory_generator]() {
        trajectory_generator.computeTimings(1e-3, 1e-3);
    };

    // for(auto eps: epsilons) {
    //  runner = eps.second;
    //
    //  clock.reset();
    //  logger.reset();
    //  logger.logExternalData("tavg_compute_param_x"+eps.first, &t_avg, 1);
    //  logger.logExternalData("iter_avg_compute_param_x"+eps.first, &iter_avg,
    //  1); trajectory_generator.add("x_traj", x_traj); run_benchmark();
    //  trajectory_generator.remove("x_traj");
    //
    //  clock.reset();
    //  logger.reset();
    //  logger.logExternalData("tavg_compute_param_y"+eps.first, &t_avg, 1);
    //  logger.logExternalData("iter_avg_compute_param_y"+eps.first, &iter_avg,
    //  1); trajectory_generator.add("y_traj", y_traj); run_benchmark();
    //  trajectory_generator.remove("y_traj");
    //
    //  clock.reset();
    //  logger.reset();
    //  logger.logExternalData("tavg_compute_param_z"+eps.first, &t_avg, 1);
    //  logger.logExternalData("iter_avg_compute_param_z"+eps.first, &iter_avg,
    //  1); trajectory_generator.add("z_traj", z_traj); run_benchmark();
    //  trajectory_generator.remove("z_traj");
    // }

    runner = epsilons["_eps_1e-6"];

    map<TrajectorySynchronization, string> sync_modes;
    sync_modes[TrajectorySynchronization::NoSynchronization] = "no_sync";
    sync_modes[TrajectorySynchronization::SynchronizeWaypoints] = "wp_sync";
    sync_modes[TrajectorySynchronization::SynchronizeTrajectory] = "traj_sync";
    // trajectory_generator.add("x_traj", x_traj);
    // trajectory_generator.add("y_traj", y_traj);
    // trajectory_generator.add("z_traj", z_traj);
    for (auto mode : sync_modes) {
        trajectory_generator.setSynchronizationMethod(mode.first);
        clock.reset();
        logger.reset();
        logger.logExternalData("tavg_compute_param_xyz_" + mode.second, &t_avg,
                               1);
        logger.logExternalData("iter_avg_compute_param_xyz_" + mode.second,
                               &iter_avg, 1);
        run_benchmark();
    }
    // trajectory_generator.remove("x_traj");
    // trajectory_generator.remove("y_traj");
    // trajectory_generator.remove("z_traj");
#endif
#if 0
	runner = std::bind(&TrajectoryGenerator::compute, &trajectory_generator);

	clock.reset();
	logger.reset();
	logger.logExternalData("tavg_compute_traj_x", &t_avg, 1);
	trajectory_generator.add("x_traj", x_traj);
	run_benchmark();
	trajectory_generator.remove("x_traj");

	clock.reset();
	logger.reset();
	logger.logExternalData("tavg_compute_traj_y", &t_avg, 1);
	trajectory_generator.add("y_traj", y_traj);
	run_benchmark();
	trajectory_generator.remove("y_traj");

	clock.reset();
	logger.reset();
	logger.logExternalData("tavg_compute_traj_z", &t_avg, 1);
	trajectory_generator.add("z_traj", z_traj);
	run_benchmark();
	trajectory_generator.remove("z_traj");

	clock.reset();
	logger.reset();
	logger.logExternalData("tavg_compute_traj_xyz", &t_avg, 1);
	trajectory_generator.add("x_traj", x_traj);
	trajectory_generator.add("y_traj", y_traj);
	trajectory_generator.add("z_traj", z_traj);
	run_benchmark();
	trajectory_generator.remove("x_traj");
	trajectory_generator.remove("y_traj");
	trajectory_generator.remove("z_traj");
#endif

    return 0;
}
