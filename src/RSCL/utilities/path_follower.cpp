#include <RSCL/utilities/path_follower.h>

using namespace RSCL;

PathFollower::PathFollower(TrajectorySynchronization sync, PathStopAction action) :
	TrajectoryGenerator(sync),
	action_(action)
{

}

void PathFollower::setStopAction(PathStopAction action) {
	action_ = action;
}

bool PathFollower::compute() {
	auto check = [](const TrajectoryParameters& param) -> bool
				 {
					 //  std::cout << *param.target_value << "," << *param.current_value << "\t";
					 return std::abs(*param.target_value - *param.current_value) < std::abs(*param.max_error);
				 };

	bool all_ok = true;
	if(action_ == PathStopAction::StopOne) {
		for(auto& params: traj_params_) {
			if(check(params.second)) {
				all_ok &= params.first->compute();
			}
			else {
				all_ok = false;
			}
		}
	}
	else {
		bool ok = true;
		for(auto& params: traj_params_) {
			if(not check(params.second)) {
				ok = false;
				break;
			}
		}
		if(ok) {
			all_ok = TrajectoryGenerator::compute();
		}
		else {
			all_ok = false;
			std::cout << "Trajectory generation paused" << std::endl;
		}
	}
	// std::cout << std::endl;
	return all_ok;
}

bool PathFollower::add(const std::string& name, TrajectoryPtr<double> item, doubleConstPtr current_value, doubleConstPtr target_value, doubleConstPtr max_error, bool force) {
	bool ok = TrajectoryGenerator::add(name, item, force);
	if(ok) {
		traj_params_[item] = {current_value, target_value, max_error};
	}
	return ok;
}

bool PathFollower::add(const std::string& name, TrajectoryPtr<double> item, const double* current_value, const double* target_value, double max_error, bool force) {
	doubleConstPtr current_value_ptr = doubleConstPtr(current_value, [](auto ptr){});
	doubleConstPtr target_value_ptr = doubleConstPtr(target_value, [](auto ptr){});
	doubleConstPtr max_error_ptr = std::make_shared<const double>(max_error);
	return add(name, item, current_value_ptr, target_value_ptr, max_error_ptr, force);
}

bool PathFollower::remove(const std::string& name) {
	TrajectoryPtr<double> traj = TrajectoryGenerator::get(name);
	bool ok = TrajectoryGenerator::remove(name);
	if(ok) {
		traj_params_.erase(traj);
	}
	return ok;
}
