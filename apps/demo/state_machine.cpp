#include "state_machine.h"


constexpr double VELOCITY_MOTION_THRESHOLD = 0.01;
constexpr double RECORD_WAYPOINT_WAIT = 3.;
constexpr double END_TEACH_WAIT = 2.*RECORD_WAYPOINT_WAIT;
constexpr double REPLAY_VMAX = 0.1;
constexpr double REPLAY_AMAX = 0.1;
constexpr double REPLAY_WMAX = 1.;
constexpr double REPLAY_DWMAX = 1.;
constexpr double FORCE_TARGET = 10.;
constexpr double FORCE_DURATION = 2.;
constexpr double STIFFNESS = 10000.;

StateMachine::StateMachine(
	RSCL::RobotPtr robot_,
	RSCL::SafetyController& controller,
	RSCL::LaserScannerDetector& laser_detector_,
	bool skip_teaching) :
	robot_(robot_),
	controller_(controller),
	laser_detector_(laser_detector_)
{
	teach_state_ = TeachStates::Init;
	replay_state_ = ReplayStates::Init;

	if(skip_teaching) {
		RSCL::Vector6d wp;
		wp << -0.146334,  -0.0682229,    0.837455,     1.57359, -0.00954553,     1.29903;
		waypoints_.push_back(wp);
		wp << -0.0908882,   -0.065789,     1.02997,     1.56203, -0.00717306,     1.29165;
		waypoints_.push_back(wp);
		wp << 0.240759, -0.0595554,   0.929419,    1.56793,  0.0133712,    1.28132;
		waypoints_.push_back(wp);
		wp << 0.193554, -0.0579581,   0.768515,    1.57518, 0.00950321,    1.27064;
		waypoints_.push_back(wp);

		teach_state_ = TeachStates::End;
	}

	trajectory_generator_ = std::make_shared<RSCL::PathFollower>(RSCL::TrajectorySynchronization::SynchronizeTrajectory, RSCL::PathStopAction::StopAll);

	operator_position_laser_ = laser_detector_.getPosition();
	operator_position_tcp_ = std::make_shared<RSCL::Vector6d>();

	max_vel_interpolator_ = std::make_shared<RSCL::LinearInterpolator>(
		std::make_shared<RSCL::LinearPoint>(0.1, 0.),     // 0m/s at 0.1m
		std::make_shared<RSCL::LinearPoint>(0.5, 0.15));   // 0.15m/s at 0.5m
	max_vel_interpolator_->enableSaturation(true);

	vmax_interpolator_output_ = max_vel_interpolator_->getOutput();

	// Fixed transformation between the laser and the arm's base. Matches the configuration on the Bazar robot_.
	laser_base_transform_ = RSCL::Matrix4d::Identity();
	RSCL::Matrix3d laser_base_rot {    Eigen::AngleAxisd   (-M_PI/2.,         RSCL::Vector3d::UnitX())
		                               * Eigen::AngleAxisd (0.,               RSCL::Vector3d::UnitY())
		                               * Eigen::AngleAxisd (-35.*M_PI/180.,   RSCL::Vector3d::UnitZ())};
	laser_base_transform_.block<3,3>(0,0) = laser_base_rot;
	laser_base_transform_.block<3,1>(0,3) = RSCL::Vector3d(+4.2856e-01, 0., +2.6822e-02);


	target_position_ = robot_->controlPointTargetPose();// std::make_shared<RSCL::Vector6d>();
	init_position_ = std::make_shared<RSCL::Vector6d>();
	traj_vel_ = std::make_shared<RSCL::Vector6d>(RSCL::Vector6d::Zero());
	stiffness_mat_ = std::make_shared<RSCL::Matrix6d>(RSCL::Matrix6d::Identity() * STIFFNESS);
	stiffness_mat_->block<3,3>(3,3).setZero();
	tcp_collision_sphere_center_ = std::make_shared<RSCL::Vector6d>(RSCL::Vector6d::Zero());
	tcp_collision_sphere_offset_ = RSCL::Vector3d(0.,0.,-7.44e-2);
	end_of_teach_ = false;
}

StateMachine::TeachStates StateMachine::getTeachState() const {
	return teach_state_;
}

StateMachine::ReplayStates StateMachine::getReplayState() const {
	return replay_state_;
}

double StateMachine::getOperatorDistance() const {
	return operator_position_tcp_->norm();
}

double StateMachine::getSeparationDistanceVelocityLimitation() const {
	return *vmax_interpolator_output_;
}

void StateMachine::init() {
	*init_position_ = *robot_->controlPointCurrentPose();

}

bool StateMachine::compute() {
	bool end = false;
	auto current_time = std::chrono::high_resolution_clock::now();
	auto dt = std::chrono::duration<double>(current_time - last_time_point_).count();

	if(not end_of_teach_) {
		double velocity = robot_->controlPointVelocity()->norm();
		switch(teach_state_) {
		case TeachStates::Init:
		{
			auto velocity_constraint = std::make_shared<RSCL::VelocityConstraint>(std::make_shared<double>(0.1));
			controller_.add(
				"velocity constraint",
				velocity_constraint);

			auto ext_force_generator = std::make_shared<RSCL::ForceProxy>(robot_->controlPointExternalForce());
			controller_.add(
				"ext force proxy",
				ext_force_generator);

			waypoints_.erase(waypoints_.begin(), waypoints_.end());
			last_time_point_ = std::chrono::high_resolution_clock::time_point::max();

			teach_state_ = TeachStates::WaitForMotion;
			break;
		}
		case TeachStates::WaitForMotion:
			if(velocity > VELOCITY_MOTION_THRESHOLD) {
				teach_state_ = TeachStates::Motion;
				std::cout << "Motion started" << std::endl;
			}
			else if(dt > END_TEACH_WAIT) {
				teach_state_ = TeachStates::End;
			}
			break;
		case TeachStates::Motion:
			if (velocity > VELOCITY_MOTION_THRESHOLD) {
				last_time_point_ = current_time;
			}
			else if(dt > RECORD_WAYPOINT_WAIT) {
				waypoints_.push_back(*robot_->controlPointCurrentPose());
				teach_state_ = TeachStates::WaitForMotion;
				std::cout << "Waypoint added" << std::endl;
			}
			break;
		case TeachStates::End:
			controller_.removeConstraint("velocity constraint");
			controller_.removeForceGenerator("ext force proxy");

			std::cout << "End of Teach" << std::endl;
			std::cout << "Recorded waypoints_: " << std::endl;
			for(const auto& waypoint: waypoints_) {
				std::cout << waypoint.transpose() << std::endl;
			}
			waypoints_.push_back(*init_position_);
			end_of_teach_ = true;
			break;
		}
	}
	else {
		tcp_collision_sphere_center_->block<3,1>(0,0) = robot_->controlPointCurrentPose()->block<3,1>(0,0) + robot_->transformationMatrix()->block<3,3>(0,0) * tcp_collision_sphere_offset_;
		laser_detector_.compute();
		computeLaserDistanceInTCPFrame(operator_position_tcp_);

		switch(replay_state_) {
		case ReplayStates::Init:
		{
			auto stiffness = std::make_shared<RSCL::StiffnessGenerator>(
				stiffness_mat_,
				target_position_,
				robot_->controlPointCurrentPose(),
				robot_->spatialTransformationMatrix());

			controller_.add("stiffness", stiffness);
			*target_position_ = *robot_->controlPointCurrentPose();

			auto potential_field_generator = std::make_shared<RSCL::PotentialFieldGenerator>(
				tcp_collision_sphere_center_,
				robot_->spatialTransformationMatrix());

			auto obstacle_position = std::make_shared<RSCL::Vector6d>();
			*obstacle_position << +8.1361e-02, -1.8324e-01, +9.7233e-01, 0., 0., 0.;
			auto obstacle = std::make_shared<RSCL::PotentialFieldObject>(
				RSCL::PotentialFieldType::Repulsive,
				std::make_shared<double>(300.),   // gain
				std::make_shared<double>(0.18),   // threshold distance
				obstacle_position);

			potential_field_generator->add("obstacle", obstacle);

			auto separation_dist_vel_cstr = std::make_shared<RSCL::SeparationDistanceConstraint>(
				std::make_shared<RSCL::VelocityConstraint>(vmax_interpolator_output_),
				max_vel_interpolator_);

			separation_dist_vel_cstr->add("operator", operator_position_tcp_);

			controller_.add(
				"sep dist cstr",
				separation_dist_vel_cstr);

			controller_.add(
				"potential field",
				potential_field_generator);

			std::cout << "Trajectory init done" << std::endl;
			replay_state_ = ReplayStates::SetupTrajectory;
		}
		break;
		case ReplayStates::SetupTrajectory:
		{
			(*stiffness_mat_)(0,0) = STIFFNESS;
			(*stiffness_mat_)(1,1) = STIFFNESS;
			(*stiffness_mat_)(2,2) = STIFFNESS;
			*target_position_ = *robot_->controlPointCurrentPose();
			if(setupTrajectoryGenerator(target_position_)) {
				std::cout << "Trajectory setup done" << std::endl;
				replay_state_ = ReplayStates::ReachingWaypoint;
			}
			else {
				std::cout << "No more waypoints_" << std::endl;
				replay_state_ = ReplayStates::End;
			}
		}

		break;
		case ReplayStates::ReachingWaypoint:
		{
			double error = (*target_position_ - *robot_->controlPointCurrentPose()).block<3,1>(0,0).norm();
			// std::cout << "error: " << error << std::endl;
			if(trajectory_generator_->compute() and error < 0.001) {
				std::cout << "Waypoint reached" << std::endl;
				if(waypoints_.size() > 0) {
					replay_state_ = ReplayStates::ForceControlInit;
				}
				else {
					replay_state_ = ReplayStates::End;
				}
			}
		}
		break;
		case ReplayStates::ForceControlInit:
		{

			auto velocity_constraint = std::make_shared<RSCL::VelocityConstraint>(std::make_shared<double>(0.1));
			auto acceleration_constraint = std::make_shared<RSCL::AccelerationConstraint>(std::make_shared<double>(0.5), SAMPLE_TIME);


			auto target_force = std::make_shared<RSCL::Vector6d>(RSCL::Vector6d::Zero());
			auto p_gain = std::make_shared<RSCL::Vector6d>(RSCL::Vector6d::Ones() * 0.005);
			auto d_gain = std::make_shared<RSCL::Vector6d>(RSCL::Vector6d::Ones() * 0.00005);
			auto selection = std::make_shared<RSCL::Vector6d>(RSCL::Vector6d::Zero());

			target_force->z() = FORCE_TARGET;
			selection->z() = 1.;

			auto force_control = std::make_shared<RSCL::ForceControl>(
				robot_->controlPointExternalForce(),
				target_force,
				SAMPLE_TIME,
				p_gain,
				d_gain,
				selection);

			force_control->configureFilter(SAMPLE_TIME, 0.1);

			controller_.add(
				"force control vlim",
				velocity_constraint);

			controller_.add(
				"force control alim",
				acceleration_constraint);

			controller_.add(
				"force control",
				force_control);

			(*stiffness_mat_)(0,0) = 0.; // seems to be needed because of a strange behavior in vrep
			(*stiffness_mat_)(1,1) = 0.; // seems to be needed because of a strange behavior in vrep
			(*stiffness_mat_)(2,2) = 0.;

			last_time_point_ = current_time;
			is_force_ok_ = false;

			std::cout << "Force control init done" << std::endl;

			replay_state_ = ReplayStates::ForceControl;
		}
		break;
		case ReplayStates::ForceControl:
			if(not is_force_ok_) {
				last_time_point_ = current_time;
			}

			is_force_ok_ = std::abs(robot_->controlPointExternalForce()->z() - FORCE_TARGET) < 0.5;

			if(dt > FORCE_DURATION) {
				replay_state_ = ReplayStates::ForceControlEnd;
			}
			break;
		case ReplayStates::ForceControlEnd:
			controller_.removeConstraint("force control vlim");
			controller_.removeConstraint("force control alim");
			controller_.removeVelocityGenerator("force control");

			std::cout << "End of force control" << std::endl;
			replay_state_ = ReplayStates::SetupTrajectory;
			break;
		case ReplayStates::End:
			controller_.removeForceGenerator("stiffness");
			controller_.removeConstraint("sep dist cstr");
			controller_.removeForceGenerator("potential field");
			std::cout << "End of replay" << std::endl;
			end = true;
			break;
		}
	}

	return end;
}

bool StateMachine::setupTrajectoryGenerator(RSCL::Vector6dPtr target_pose) {
	if(waypoints_.size() == 0)
		return false;

	auto to = waypoints_.front();
	waypoints_.pop_front();
	std::cout << "Going from [" << robot_->controlPointCurrentPose()->transpose() << "] to [" << to.transpose() << "]" << std::endl;

	trajectory_generator_->removeAll();

	for (size_t i = 0; i < 6; ++i) {
		auto point_from = std::make_shared<RSCL::TrajectoryPoint>((*robot_->controlPointCurrentPose())(i), 0., 0.);
		auto point_to = std::make_shared<RSCL::TrajectoryPoint>(to(i), 0., 0.);
		auto trajectory = std::make_shared<RSCL::Trajectory>(RSCL::TrajectoryOutputType::Position, point_from, target_pose->data()+i, SAMPLE_TIME);
		if(i < 3) {
			trajectory->addPathTo(point_to, REPLAY_VMAX, REPLAY_AMAX);
		}
		else {
			trajectory->addPathTo(point_to, REPLAY_WMAX, REPLAY_DWMAX);
		}

		trajectory_generator_->add("traj"+std::to_string(i), trajectory, robot_->controlPointCurrentPose()->data()+i, target_pose->data()+i, 0.05);
	}

	trajectory_generator_->computeParameters();

	return true;
}

void StateMachine::computeLaserDistanceInTCPFrame(RSCL::Vector6dPtr obstacle_position) {
	RSCL::Vector4d position_laser = RSCL::Vector4d::Ones();
	position_laser.x() = operator_position_laser_->x();
	position_laser.y() = operator_position_laser_->y();
	position_laser.z() = 0.;

	RSCL::Vector4d position_base = laser_base_transform_ * position_laser;

	const auto& tcp_base_transform = *robot_->transformationMatrix();
	RSCL::Matrix4d base_tcp_transform = RSCL::Matrix4d::Identity();
	base_tcp_transform.block<3,3>(0,0) = tcp_base_transform.block<3,3>(0,0).transpose();
	base_tcp_transform.block<3,1>(0,3) = -tcp_base_transform.block<3,3>(0,0).transpose() * tcp_base_transform.block<3,1>(0,3);

	RSCL::Vector4d position_tcp = base_tcp_transform * position_base;

	// The following is to remove the vertical component from the position
	RSCL::Vector3d position_tcp_rbase = tcp_base_transform.block<3,3>(0,0) * position_tcp.block<3,1>(0,0);
	position_tcp_rbase.y() = 0.;

	obstacle_position->setZero();
	obstacle_position->block<3,1>(0,0) = tcp_base_transform.block<3,3>(0,0).transpose() * position_tcp_rbase;
}
