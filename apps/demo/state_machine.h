#pragma once

#include <OpenPHRI/OpenPHRI.h>
#include <chrono>
#include <list>

class StateMachine {
public:
	enum class InitStates {
		SetupTrajectory,
		GoToInitPose,
		InitPoseReached
	};

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
		std::shared_ptr<phri::LaserScannerDetector> laser_detector,
		bool skip_teaching = false);
	~StateMachine() = default;

	// Read init position, set parameters
	void init();

	void useNullSpaceMotion(double null_space_gain, phri::VectorXd&& joint_min_positions, phri::VectorXd&& joint_max_positions);

	// return true when finished
	bool compute();

	bool goToInitPose();

	TeachStates getTeachState() const;
	ReplayStates getReplayState() const;

	double getOperatorDistance() const;
	double getSeparationDistanceVelocityLimitation() const;

private:
	bool setupTrajectoryGenerator();
	bool setupJointTrajectoryGenerator();
	void computeLaserDistanceInTCPFrame(phri::Vector6dPtr obstacle_position);
	void computeNullSpaceVelocityVector();

	phri::RobotPtr robot_;
	phri::SafetyController& controller_;
	std::shared_ptr<phri::LaserScannerDetector> laser_detector_;

	InitStates init_state_;
	TeachStates teach_state_;
	ReplayStates replay_state_;
	std::list<phri::Vector6d> waypoints_;
	phri::TrajectoryPoint<phri::VectorXd> joint_start_point_;
	phri::TrajectoryPoint<phri::VectorXd> joint_end_point_;
	phri::TrajectoryPoint<phri::Vector6d> start_point_;
	phri::TrajectoryPoint<phri::Vector6d> end_point_;
	std::shared_ptr<phri::TrajectoryGenerator<phri::Vector6d>> trajectory_generator_;
	std::shared_ptr<phri::TrajectoryGenerator<phri::VectorXd>> joint_trajectory_generator_;
	std::shared_ptr<phri::LinearInterpolator> max_vel_interpolator_;
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
	bool force_control_at_next_wp_;

	std::chrono::high_resolution_clock::time_point last_time_point_;
	bool is_force_ok_;

	std::shared_ptr<phri::VectorXd> null_space_velocity_;
	phri::VectorXd joint_min_positions_;
	phri::VectorXd joint_max_positions_;
	double null_space_gain_;

	bool is_robot_stopped_;
};
