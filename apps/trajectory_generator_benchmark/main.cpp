#include <iostream>
#include <type_traits>

#include <RSCL/RSCL.h>

using namespace std;
using namespace RSCL;

constexpr double SAMPLE_TIME = 0.010;
constexpr bool USE_LOOP = false;

int main(int argc, char const *argv[]) {
	auto x_point_1 = make_shared<TrajectoryPoint<double>>(0,    0.,     0.);
	auto x_point_2 = make_shared<TrajectoryPoint<double>>(0.1,  0.,     0.);
	auto x_point_3 = make_shared<TrajectoryPoint<double>>(0.3,  0.,     0.);

	auto y_point_1 = make_shared<TrajectoryPoint<double>>(0.,   0.,     0.);
	auto y_point_2 = make_shared<TrajectoryPoint<double>>(0.2, 0.1,    0.05);
	auto y_point_3 = make_shared<TrajectoryPoint<double>>(0.3,  0.,     0.);

	auto z_point_1 = make_shared<TrajectoryPoint<double>>(0.,   0.1,    0.1);
	auto z_point_2 = make_shared<TrajectoryPoint<double>>(0.2,  0.05,   -0.05);
	auto z_point_3 = make_shared<TrajectoryPoint<double>>(0.1,  -0.1,    0.1);

	double x_vel, y_vel, z_vel;

	auto x_traj = make_shared<Trajectory<double>>(TrajectoryOutputType::Velocity, x_point_1, &x_vel, SAMPLE_TIME);
	auto y_traj = make_shared<Trajectory<double>>(TrajectoryOutputType::Velocity, y_point_1, &y_vel, SAMPLE_TIME);
	auto z_traj = make_shared<Trajectory<double>>(TrajectoryOutputType::Velocity, z_point_1, &z_vel, SAMPLE_TIME);


	x_traj->addPathTo(x_point_2, 0.05, 0.01);
	x_traj->addPathTo(x_point_3, 0.1, 0.02);
	x_traj->addPathTo(x_point_1, 0.05, 0.01);
	x_traj->addPathTo(x_point_3, 0.1, 0.02);
	x_traj->addPathTo(x_point_2, 0.05, 0.01);
	x_traj->addPathTo(x_point_1, 0.05, 0.01);


	y_traj->addPathTo(y_point_2, 0.15, 0.1);
	y_traj->addPathTo(y_point_3, 0.25, 0.2);
	y_traj->addPathTo(y_point_1, 0.15, 0.1);
	y_traj->addPathTo(y_point_3, 0.25, 0.2);
	y_traj->addPathTo(y_point_2, 0.15, 0.1);
	y_traj->addPathTo(y_point_1, 0.25, 0.2);

	z_traj->addPathTo(z_point_2, 0.2, 0.2);
	z_traj->addPathTo(z_point_3, 0.2, 0.2);
	z_traj->addPathTo(z_point_1, 0.2, 0.2);
	z_traj->addPathTo(z_point_3, 0.2, 0.2);
	z_traj->addPathTo(z_point_2, 0.2, 0.2);
	z_traj->addPathTo(z_point_1, 0.2, 0.2);

	// Use these for fixed-time paths
	// z_traj->addPathTo(z_point_2, 5.);
	// z_traj->addPathTo(z_point_3, 5.);
	// z_traj->addPathTo(z_point_1, 10.);

	auto trajectory_generator = TrajectoryGenerator(TrajectorySynchronization::NoSynchronization);
	// trajectory_generator.add("y_traj", y_traj);
	// trajectory_generator.add("z_traj", z_traj);

	// constexpr size_t N = 100000;
	Clock clock;
	DataLogger logger(
		"/mnt/tmpfs/rscl_logs",
		clock.getTime(),
		true,   // create gnuplot files
		true);  // delay disk write

	double t_start, t_end, t_avg = 0., iter_avg = 0.;
	constexpr int NCalls = 900;
	constexpr int Tries = 100;
	auto getAvgTime = [&t_start, &t_end, NCalls](){return (t_end-t_start)/double(NCalls);};
	std::function<void(void)> runner = std::bind(&TrajectoryGenerator::computeParameters, &trajectory_generator, 1e-6, 1e-6);

	auto run_benchmark =
		[&]() {
			std::cout << "Benchmark started" << std::endl;
			for (size_t i = 0; i < Tries; ++i) {
				Trajectory<double>::resetComputeTimingsIterations();
				if(USE_LOOP) {
					for (size_t j = 0; j < NCalls; ++j) {
						t_start = clock();
						runner();
						t_end = clock();
						t_avg += t_end - t_start;
					}
					t_avg /= double(NCalls);
				}
				else {
					t_start = clock();
					run_function<NCalls>(runner);
					t_end = clock();
					t_avg = getAvgTime();
				}
				t_avg *= 1e6;
				iter_avg = double(Trajectory<double>::getComputeTimingsIterations()) / double(NCalls);
				// std::cout << "Average time (us): " << t_avg << std::endl;
				logger();
			}
			std::cout << "Benchmark ended" << std::endl;
		};

	// Warm up
	std::cout << "Warming up" << std::endl;
	clock.reset();
	while(clock() < 3.) continue;

#if 1
	map<string, std::function<void(void)>> epsilons;
	epsilons["_eps_1e-6"] = std::bind(&TrajectoryGenerator::computeParameters, &trajectory_generator, 1e-6, 1e-6);
	epsilons["_eps_1e-3"] = std::bind(&TrajectoryGenerator::computeParameters, &trajectory_generator, 1e-3, 1e-3);

	for(auto eps: epsilons) {
		runner = eps.second;

		clock.reset();
		logger.reset();
		logger.logExternalData("tavg_compute_param_x"+eps.first, &t_avg, 1);
		logger.logExternalData("iter_avg_compute_param_x"+eps.first, &iter_avg, 1);
		trajectory_generator.add("x_traj", x_traj);
		run_benchmark();
		trajectory_generator.remove("x_traj");

		clock.reset();
		logger.reset();
		logger.logExternalData("tavg_compute_param_y"+eps.first, &t_avg, 1);
		logger.logExternalData("iter_avg_compute_param_y"+eps.first, &iter_avg, 1);
		trajectory_generator.add("y_traj", y_traj);
		run_benchmark();
		trajectory_generator.remove("y_traj");

		clock.reset();
		logger.reset();
		logger.logExternalData("tavg_compute_param_z"+eps.first, &t_avg, 1);
		logger.logExternalData("iter_avg_compute_param_z"+eps.first, &iter_avg, 1);
		trajectory_generator.add("z_traj", z_traj);
		run_benchmark();
		trajectory_generator.remove("z_traj");
	}

	runner = epsilons["_eps_1e-6"];

	map<TrajectorySynchronization, string> sync_modes;
	sync_modes[TrajectorySynchronization::NoSynchronization] = "no_sync";
	sync_modes[TrajectorySynchronization::SynchronizeWaypoints] = "wp_sync";
	sync_modes[TrajectorySynchronization::SynchronizeTrajectory] = "traj_sync";
	trajectory_generator.add("x_traj", x_traj);
	trajectory_generator.add("y_traj", y_traj);
	trajectory_generator.add("z_traj", z_traj);
	for(auto mode: sync_modes) {
		trajectory_generator.setSynchronizationMethod(mode.first);
		clock.reset();
		logger.reset();
		logger.logExternalData("tavg_compute_param_xyz_"+mode.second, &t_avg, 1);
		logger.logExternalData("iter_avg_compute_param_xyz_"+mode.second, &iter_avg, 1);
		run_benchmark();
	}
	trajectory_generator.remove("x_traj");
	trajectory_generator.remove("y_traj");
	trajectory_generator.remove("z_traj");
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
