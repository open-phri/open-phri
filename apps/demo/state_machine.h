#pragma once

#include <RSCL/RSCL.h>
#include <chrono>
#include <list>

constexpr double SAMPLE_TIME = 0.010;

class StateMachine {
public:
	enum class TeachStates {
		Init,
		WaitForMotion,
		Motion,
		End
	};

	enum class ReplayStates {
		Init,
		SetupTrajectory,
		ReachingWaypoint,
		ForceControlInit,
		ForceControl,
		ForceControlEnd,
		End
	};


	StateMachine(
		RSCL::RobotPtr robot,
		RSCL::SafetyController& controller,
		RSCL::LaserScannerDetector& laser_detector,
		bool skip_teaching = false);
	~StateMachine() = default;

	// Read init position, set parameters
	void init();

	// return true when finished
	bool compute();

	TeachStates getTeachState() const;
	ReplayStates getReplayState() const;

private:
	bool setupTrajectoryGenerator(RSCL::Vector6dPtr target_pose);
	void computeLaserDistanceInTCPFrame(RSCL::Vector6dPtr obstacle_position);

	RSCL::RobotPtr robot_;
	RSCL::SafetyController& controller_;
	RSCL::LaserScannerDetector& laser_detector_;

	TeachStates teach_state_;
	ReplayStates replay_state_;
	std::list<RSCL::Vector6d> waypoints_;
	std::shared_ptr<RSCL::PathFollower> trajectory_generator_;
	std::shared_ptr<RSCL::LinearInterpolator> max_vel_interpolator_;
	std::shared_ptr<RSCL::Vector6d> target_position_;
	std::shared_ptr<RSCL::Vector6d> init_position_;
	std::shared_ptr<RSCL::Vector6d> traj_vel_;
	std::shared_ptr<RSCL::Matrix6d> stiffness_mat_;
	std::shared_ptr<const double> vmax_interpolator_output_;
	std::shared_ptr<const RSCL::Vector2d> operator_position_laser_;
	std::shared_ptr<RSCL::Vector6d> operator_position_tcp_;
	std::shared_ptr<RSCL::Vector6d> tcp_collision_sphere_center_;
	RSCL::Vector3d tcp_collision_sphere_offset_;
	RSCL::Matrix4d laser_base_transform_;
	bool end_of_teach_;

	std::chrono::high_resolution_clock::time_point last_time_point_;
	bool is_force_ok_;
};
