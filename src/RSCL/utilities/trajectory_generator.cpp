#include <RSCL/utilities/trajectory_generator.h>
#include <iostream>

using namespace RSCL;


namespace RSCL {
size_t _compute_timings_total_iter = 0;             // for benchmarking only

template class Trajectory<double>;
template class Trajectory<VectorXd>;
template class Trajectory<Vector6d>;

}

TrajectoryGenerator::TrajectoryGenerator(TrajectorySynchronization sync)
{
	setSynchronizationMethod(sync);
}

void TrajectoryGenerator::setSynchronizationMethod(TrajectorySynchronization sync) {
	sync_ = sync;
}

void TrajectoryGenerator::computeParameters(double v_eps, double a_eps) {
	// Have not seen the benefit of using OpenMP for this part, it is slower or equivalent to version below
	for(const auto& traj: items_) {
		bool ok = traj.second->computeTimings(v_eps,a_eps);
		if(not ok) {
			std::cerr << "In TrajectoryGenerator::computeParameters: couldn't compute timings for trajectory " << traj.first << std::endl;
		}
	}

	if(sync_ == TrajectorySynchronization::SynchronizeWaypoints) {
		size_t max_segments = 0;
		for(auto& traj: items_) {
			max_segments = std::max(max_segments, traj.second->getSegmentCount());
		}

		for (size_t idx = 0; idx < max_segments; ++idx) {
			double max_time = 0.;
			for(auto& traj: items_) {
				max_time = std::max(max_time, traj.second->getPathMinimumTime(idx));
			}
			for(auto& traj: items_) {
				traj.second->setPaddingTime(idx, max_time - traj.second->getPathMinimumTime(idx));
			}
		}
	}
	else if(sync_ == TrajectorySynchronization::SynchronizeTrajectory) {
		double max_time = 0.;
		for(auto& traj: items_) {
			max_time = std::max(max_time, traj.second->getTrajectoryMinimumTime());
		}
		for(auto& traj: items_) {
			size_t segments = traj.second->getSegmentCount();
			double padding = (max_time - traj.second->getTrajectoryMinimumTime())/double(segments);
			for (size_t idx = 0; idx < segments; ++idx) {
				traj.second->setPaddingTime(idx, padding);
			}
		}
	}
	else {
		for(auto& traj: items_) {
			for (size_t idx = 0; idx < traj.second->getSegmentCount(); ++idx) {
				traj.second->setPaddingTime(idx, 0.);
			}
		}
	}

	for(auto& traj: items_) {
		traj.second->computeParameters();
	}
}

bool TrajectoryGenerator::compute() {
	bool all_ok = true;

	for(auto& traj: items_) {
		all_ok &= traj.second->compute();
	}

	return all_ok;
}

double TrajectoryGenerator::operator()() {
	return compute();
}

void TrajectoryGenerator::reset() {
	for(auto& traj: items_) {
		traj.second->reset();
	}
}
