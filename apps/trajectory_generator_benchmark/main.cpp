#include <iostream>
#include <ctime>
#include <type_traits>

#include <RSCL/RSCL.h>

using namespace std;
using namespace RSCL;

constexpr double SAMPLE_TIME = 0.010;

template<int N>
void run_once(std::function<void(void)> todo) {
	todo();
	run_once<N-1>(todo);
}

template<>
void run_once<0>(std::function<void(void)> todo) {
	return;
}

template<int N, typename std::enable_if<(N <= 900), bool>::type = 0>
void run(std::function<void(void)> todo) {
	run_once<N>(todo);
}

template<int N, typename std::enable_if<not (N <= 900), bool>::type = 0> // use <= to remove parsing error in atom when using >
void run(std::function<void(void)> todo) {
	size_t iter = N;
	while(iter >= 900) {
		run_once<900>(todo);
		iter -= 900;
	}
	run_once<N%900>(todo);
}



int main(int argc, char const *argv[]) {
	auto x_point_1 = make_shared<TrajectoryPoint>(-0.197,  0.,      0.);
	auto x_point_2 = make_shared<TrajectoryPoint>(-0.25,    0.01,  -0.01);
	auto x_point_3 = make_shared<TrajectoryPoint>(-0.15,    0.,     0.);

	auto z_point_1 = make_shared<TrajectoryPoint>(1.1249,  0.,     0.);
	auto z_point_2 = make_shared<TrajectoryPoint>(1.,      -0.025, 0.);
	auto z_point_3 = make_shared<TrajectoryPoint>(0.85,    0.,     0.);

	double x_vel, z_vel;

	auto x_traj = make_shared<Trajectory>(TrajectoryOutputType::Velocity, x_point_1, &x_vel, SAMPLE_TIME);
	auto z_traj = make_shared<Trajectory>(TrajectoryOutputType::Velocity, z_point_1, &z_vel, SAMPLE_TIME);


	x_traj->addPathTo(x_point_2, 0.05, 0.01);
	x_traj->addPathTo(x_point_3, 0.1, 0.02);
	x_traj->addPathTo(x_point_1, 0.05, 0.01);
	x_traj->addPathTo(x_point_3, 0.1, 0.02);
	x_traj->addPathTo(x_point_2, 0.05, 0.01);
	x_traj->addPathTo(x_point_1, 0.05, 0.01);

	z_traj->addPathTo(z_point_2, 0.05, 0.1);
	z_traj->addPathTo(z_point_3, 0.05, 0.2);
	z_traj->addPathTo(z_point_1, 0.05, 0.1);
	z_traj->addPathTo(z_point_3, 0.05, 0.2);
	z_traj->addPathTo(z_point_2, 0.05, 0.1);
	z_traj->addPathTo(z_point_1, 0.05, 0.1);

	// Use these for fixed-time paths
	// z_traj->addPathTo(z_point_2, 5.);
	// z_traj->addPathTo(z_point_3, 5.);
	// z_traj->addPathTo(z_point_1, 10.);

	auto trajectory_generator = TrajectoryGenerator(TrajectorySynchronization::SynchronizeTrajectory);
	trajectory_generator.add("x_traj", x_traj);
	trajectory_generator.add("z_traj", z_traj);

	double t_avg = 0;
	constexpr size_t N = 100000;
	extern size_t _compute_timings_total_iter;

	std::function<void(void)> f = std::bind(&TrajectoryGenerator::computeParameters, &trajectory_generator);
	clock_t begin = clock();
	run<N>(f);
	clock_t end = clock();
	t_avg = double(end - begin) / double(N * CLOCKS_PER_SEC);

	cout << "Average time for " << N << " computations: " << t_avg << endl;
	cout << "Total iteration count: \t" << _compute_timings_total_iter << endl;
	cout << "x trajectory\n\ttotal time: \t" << x_traj->getTrajectoryDuration() << endl;
	cout << "\tminimum time: \t" << x_traj->getTrajectoryMinimumTime() << endl;
	cout << "z trajectory\n\ttotal time: \t" << z_traj->getTrajectoryDuration() << endl;
	cout << "\tminimum time: \t" << z_traj->getTrajectoryMinimumTime() << endl;

	return 0;
}
