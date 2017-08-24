#pragma once

#include <OpenPHRI/OpenPHRI.h>
#include <chrono>
#include <list>

constexpr double SAMPLE_TIME = 0.005;

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
		phri::RobotPtr robot,
		phri::SafetyController& controller,
		phri::LaserScannerDetector& laser_detector,
		bool skip_teaching = false);
	~StateMachine() = default;

	// Read init position, set parameters
	void init();

	// return true when finished
	bool compute();

	TeachStates getTeachState() const;
	ReplayStates getReplayState() const;

	double getOperatorDistance() const;
	double getSeparationDistanceVelocityLimitation() const;

private:
	bool setupTrajectoryGenerator();
	void computeLaserDistanceInTCPFrame(phri::Vector6dPtr obstacle_position);

	phri::RobotPtr robot_;
	phri::SafetyController& controller_;
	phri::LaserScannerDetector& laser_detector_;
	std::shared_ptr<phri::Integrator<phri::Vector6d>> target_integrator_;

	TeachStates teach_state_;
	ReplayStates replay_state_;
	std::list<phri::Vector6d> waypoints_;
	std::shared_ptr<phri::TrajectoryGenerator> trajectory_generator_;
	std::shared_ptr<phri::LinearInterpolator> max_vel_interpolator_;
	std::shared_ptr<phri::Vector6d> target_velocity_;
	std::shared_ptr<phri::Vector6d> init_position_;
	std::shared_ptr<phri::Vector6d> traj_vel_;
	std::shared_ptr<phri::Matrix6d> stiffness_mat_;
	std::shared_ptr<const double> vmax_interpolator_output_;
	std::shared_ptr<const phri::Vector2d> operator_position_laser_;
	std::shared_ptr<phri::Vector6d> operator_position_tcp_;
	std::shared_ptr<phri::Vector6d> tcp_collision_sphere_center_;
	phri::Vector3d tcp_collision_sphere_offset_;
	phri::Matrix4d laser_base_transform_;
	bool end_of_teach_;

	std::chrono::high_resolution_clock::time_point last_time_point_;
	bool is_force_ok_;
};
