#include <iostream>

#include <OpenPHRI/OpenPHRI.h>

using namespace std;
using namespace phri;

constexpr double SAMPLE_TIME = 0.001;

int main(int argc, char const *argv[]) {

	std::cout << "Blah?\n";
	auto foo = make_shared<TrajectoryPoint<Vector6d>>(Vector6d::Zero(), Vector6d::Zero(), Vector6d::Zero());
	auto foofoo = make_shared<TrajectoryPoint<Vector6d>>(Vector6d::Ones(), Vector6d::Zero(), Vector6d::Zero());
	auto bar = Trajectory<Vector6d>(TrajectoryOutputType::Position, foo, SAMPLE_TIME);
	bar.addPathTo(foofoo, Vector6d::Ones(), Vector6d::Ones());
	bar.computeParameters();

	auto x_point_1 = make_shared<TrajectoryPoint<double>>(0.,  0.,     0.);
	auto x_point_2 = make_shared<TrajectoryPoint<double>>(1.,  0.,     0.);
	auto x_point_3 = make_shared<TrajectoryPoint<double>>(-1,  0.,     0.);

	auto y_point_1 = make_shared<TrajectoryPoint<double>>(1.,  0.,  0.);
	auto y_point_2 = make_shared<TrajectoryPoint<double>>(-1., 0.,  0.);
	auto y_point_3 = make_shared<TrajectoryPoint<double>>(0.,  0.,  0.);

	auto target_position = Vector2d();

	auto x_traj = make_shared<Trajectory<double>>(TrajectoryOutputType::Position, x_point_1, &(target_position.x()), SAMPLE_TIME);
	auto y_traj = make_shared<Trajectory<double>>(TrajectoryOutputType::Position, y_point_1, &(target_position.y()), SAMPLE_TIME);


	x_traj->addPathTo(x_point_2, 0.5, 1.);
	x_traj->addPathTo(x_point_3, 0.5, 1.);

	y_traj->addPathTo(y_point_2, 0.5, 0.5);
	y_traj->addPathTo(y_point_3, 0.25, 0.5);

	auto trajectory_generator = TrajectoryGenerator(TrajectorySynchronization::NoSynchronization);
	trajectory_generator.add("x_traj", x_traj);
	trajectory_generator.add("y_traj", y_traj);

	auto print_durations = [&x_traj, &y_traj]() {
							   std::cout << "-------------------------------------\n";
							   std::cout << "x_traj: 0., " << x_traj->getPathDuration(0) << ", " << x_traj->getPathDuration(0)+x_traj->getPathDuration(1) << "\n";
							   std::cout << "y_traj: 0., " << y_traj->getPathDuration(0) << ", " << y_traj->getPathDuration(0)+y_traj->getPathDuration(1) << "\n";
						   };

	trajectory_generator.computeParameters();
	print_durations();

	Clock clock(SAMPLE_TIME);
	DataLogger logger(
		"/mnt/tmpfs/open-phri_logs",
		clock.getTime(),
		true, // create gnuplot files
		true);

	logger.logExternalData("trajectory_no_sync", target_position.data(), 2);
	do {
		clock();
		logger();
	} while(not trajectory_generator.compute());

	logger.reset();
	clock.reset();
	logger.logExternalData("trajectory_wp_sync", target_position.data(), 2);
	trajectory_generator.setSynchronizationMethod(TrajectorySynchronization::SynchronizeWaypoints);
	trajectory_generator.computeParameters();
	trajectory_generator.reset();
	print_durations();
	do {
		clock();
		logger();
	} while(not trajectory_generator.compute());

	logger.reset();
	clock.reset();
	logger.logExternalData("trajectory_traj_sync", target_position.data(), 2);
	trajectory_generator.setSynchronizationMethod(TrajectorySynchronization::SynchronizeTrajectory);
	trajectory_generator.computeParameters();
	trajectory_generator.reset();
	print_durations();
	do {
		clock();
		logger();
	} while(not trajectory_generator.compute());



	return 0;
}
