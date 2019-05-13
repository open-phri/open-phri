/*      File: state_machine.cpp
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

#include "state_machine.h"

constexpr double VELOCITY_MOTION_THRESHOLD = 0.01;
constexpr double RECORD_WAYPOINT_WAIT = 3.;
constexpr double END_TEACH_WAIT = 2. * RECORD_WAYPOINT_WAIT;
constexpr double REPLAY_VMAX = 0.1;
constexpr double REPLAY_AMAX = 0.1;
constexpr double REPLAY_WMAX = 1.;
constexpr double REPLAY_DWMAX = 1.;
constexpr double REPLAY_DQMAX = 1.;
constexpr double REPLAY_D2QMAX = 0.3;
constexpr double REPLAY_ERROR_THRESHOLD_TRANS = 0.01;
constexpr double REPLAY_ERROR_THRESHOLD_ROT = 100.;
constexpr double FORCE_DURATION = 2.;
constexpr double FORCE_TARGET = 10.;
constexpr double FORCE_CTRL_VLIM = 0.1;
constexpr double FORCE_CTRL_ALIM = 0.5;
constexpr double FORCE_CTRL_PGAIN = 0.005;
constexpr double FORCE_CTRL_DGAIN = 0.0005;
constexpr double STIFFNESS = 100.;

extern double SAMPLE_TIME;

StateMachine::StateMachine(
    phri::RobotPtr robot, phri::SafetyController& controller,
    std::shared_ptr<phri::LaserScannerDetector> laser_detector_,
    bool skip_teaching)
    : robot_(robot),
      controller_(controller),
      laser_detector_(laser_detector_),
      force_control_at_next_wp_(true),
      is_robot_stopped_(false) {
    init_state_ = InitStates::SetupTrajectory;
    teach_state_ = TeachStates::Init;
    replay_state_ = ReplayStates::Init;

    if (skip_teaching) {
        auto q = Eigen::Quaterniond(0.499977, 0.500029, -0.499972, 0.500021);
        phri::Vector3d wp;
        wp << -0.176542, -0.0672858, 0.772875;
        waypoints_.emplace_back(wp, q);
        waypoints_.emplace_back(wp, q);
        wp << -0.173713, -0.0660992, 0.959651;
        waypoints_.emplace_back(wp, q);
        waypoints_.emplace_back(wp, q);
        wp << 0.156179, -0.0612425, 0.949056;
        waypoints_.emplace_back(wp, q);
        waypoints_.emplace_back(wp, q);
        wp << 0.150482, -0.0590984, 0.777583;
        waypoints_.emplace_back(wp, q);
        waypoints_.emplace_back(wp, q);
        teach_state_ = TeachStates::End;
    }

    joint_trajectory_generator_ =
        std::make_shared<phri::TrajectoryGenerator<phri::VectorXd>>(
            joint_start_point_, SAMPLE_TIME,
            phri::TrajectorySynchronization::SynchronizeTrajectory);
    phri::VectorXd dqmax(robot_->jointCount()), d2qmax(robot_->jointCount());
    dqmax.setConstant(REPLAY_DQMAX);
    d2qmax.setConstant(REPLAY_D2QMAX);
    joint_trajectory_generator_->addPathTo(joint_end_point_, dqmax, d2qmax);

    trajectory_generator_ =
        std::make_shared<phri::TrajectoryGenerator<phri::Pose>>(
            start_point_, SAMPLE_TIME,
            phri::TrajectorySynchronization::SynchronizeTrajectory);
    phri::Vector6d err_th;
    phri::Twist vmax(phri::Vector3d::Ones() * REPLAY_VMAX,
                     phri::Vector3d::Ones() * REPLAY_WMAX);
    phri::Acceleration amax(phri::Vector3d::Ones() * REPLAY_AMAX,
                            phri::Vector3d::Ones() * REPLAY_DWMAX);
    trajectory_generator_->addPathTo(end_point_, vmax, amax);
    err_th.block<3, 1>(0, 0).setConstant(REPLAY_ERROR_THRESHOLD_TRANS);
    err_th.block<3, 1>(3, 0).setConstant(REPLAY_ERROR_THRESHOLD_ROT);
    trajectory_generator_->enableErrorTracking(
        robot_->controlPointCurrentPose(), err_th, true);

    if (static_cast<bool>(laser_detector_)) {
        operator_position_laser_ = laser_detector_->getPosition();
    } else {
        operator_position_laser_ = std::make_shared<phri::Vector2d>(3., 0.);
    }
    operator_position_tcp_ = std::make_shared<phri::Pose>();

    max_vel_interpolator_ = std::make_shared<phri::LinearInterpolator>(
        std::make_shared<phri::LinearPoint>(0.1, 0.),    // 0m/s at 0.1m
        std::make_shared<phri::LinearPoint>(0.5, 0.15)); // 0.15m/s at 0.5m
    max_vel_interpolator_->enableSaturation(true);

    vmax_interpolator_output_ = max_vel_interpolator_->getOutput();

    // Fixed transformation between the laser and the arm's base. Matches the
    // configuration on the Bazar robot_.
    laser_base_transform_ = phri::Matrix4d::Identity();
    phri::Matrix3d laser_base_rot{
        Eigen::AngleAxisd(-M_PI / 2., phri::Vector3d::UnitX()) *
        Eigen::AngleAxisd(0., phri::Vector3d::UnitY()) *
        Eigen::AngleAxisd(-35. * M_PI / 180., phri::Vector3d::UnitZ())};
    laser_base_transform_.block<3, 3>(0, 0) = laser_base_rot;
    laser_base_transform_.block<3, 1>(0, 3) =
        phri::Vector3d(+4.2856e-01, 0., +2.6822e-02);

    init_position_ = std::make_shared<phri::Pose>();
    traj_vel_ = std::make_shared<phri::Twist>();
    stiffness_mat_ = std::make_shared<phri::Matrix6d>(
        phri::Matrix6d::Identity() * STIFFNESS);
    // stiffness_mat_->block<3,3>(3,3).setZero();
    (*stiffness_mat_)(4, 4) =
        0.; // TODO it works but it should be z axis, fix it
    tcp_collision_sphere_center_ = std::make_shared<phri::Pose>();
    tcp_collision_sphere_offset_ = phri::Vector3d(0., 0., -7.44e-2);
    end_of_teach_ = false;
}

StateMachine::TeachStates StateMachine::getTeachState() const {
    return teach_state_;
}

StateMachine::ReplayStates StateMachine::getReplayState() const {
    return replay_state_;
}

double StateMachine::getOperatorDistance() const {
    return operator_position_tcp_->translation().norm();
}

double StateMachine::getSeparationDistanceVelocityLimitation() const {
    return *vmax_interpolator_output_;
}

void StateMachine::init() {
    robot_->controlPointDampingMatrix()->setOnes();
    robot_->controlPointDampingMatrix()->block<3, 1>(0, 0) *= 250.;
    robot_->controlPointDampingMatrix()->block<3, 1>(3, 0) *= 500.;
    *init_position_ = *robot_->controlPointCurrentPose();
    *robot_->jointTargetPosition() = *robot_->jointCurrentPosition();
}

bool StateMachine::compute() {
    bool end = false;
    auto current_time = std::chrono::high_resolution_clock::now();
    auto dt =
        std::chrono::duration<double>(current_time - last_time_point_).count();

    if (not end_of_teach_) {
        double velocity =
            static_cast<phri::Vector6d>(*robot_->controlPointVelocity()).norm();
        switch (teach_state_) {
        case TeachStates::Init: {
            auto velocity_constraint =
                std::make_shared<phri::VelocityConstraint>(
                    std::make_shared<double>(0.1));
            controller_.add("velocity constraint", velocity_constraint);

            auto ext_force_generator = std::make_shared<phri::ForceProxy>(
                robot_->controlPointExternalForce());
            controller_.add("ext force proxy", ext_force_generator);

            waypoints_.erase(waypoints_.begin(), waypoints_.end());
            last_time_point_ =
                std::chrono::high_resolution_clock::time_point::max();

            teach_state_ = TeachStates::WaitForMotion;
            break;
        }
        case TeachStates::WaitForMotion:
            if (velocity > VELOCITY_MOTION_THRESHOLD) {
                teach_state_ = TeachStates::Motion;
                std::cout << "Motion started" << std::endl;
            } else if (dt > END_TEACH_WAIT) {
                teach_state_ = TeachStates::End;
            }
            break;
        case TeachStates::Motion:
            if (velocity > VELOCITY_MOTION_THRESHOLD) {
                last_time_point_ = current_time;
            } else if (dt > RECORD_WAYPOINT_WAIT) {
                waypoints_.push_back(*robot_->controlPointCurrentPose());
                waypoints_.push_back(*robot_->controlPointCurrentPose());
                teach_state_ = TeachStates::WaitForMotion;
                std::cout << "Waypoint added" << std::endl;
            }
            break;
        case TeachStates::End:
            // Will fail if the teaching has been skipped, so just discard the
            // error
            try {
                controller_.removeConstraint("velocity constraint");
                controller_.removeForceGenerator("ext force proxy");
            } catch (std::domain_error& err) {
            }

            std::cout << "End of Teach" << std::endl;
            std::cout << "Recorded waypoints: " << std::endl;
            for (const auto& waypoint : waypoints_) {
                std::cout << waypoint << std::endl;
            }
            waypoints_.push_back(*init_position_);
            end_of_teach_ = true;
            break;
        }
    } else {
        if (static_cast<bool>(laser_detector_)) {
            laser_detector_->compute();
        }
        computeLaserDistanceInTCPFrame(operator_position_tcp_);

        switch (replay_state_) {
        case ReplayStates::Init: {
            auto stiffness = std::make_shared<phri::StiffnessGenerator>(
                stiffness_mat_, robot_->controlPointTargetPose(),
                phri::ReferenceFrame::TCP);

            controller_.add("stiffness", stiffness);
            *robot_->controlPointTargetPose() =
                *robot_->controlPointCurrentPose();

            controller_.add(
                "traj vel",
                phri::VelocityProxy(trajectory_generator_->getVelocityOutput(),
                                    phri::ReferenceFrame::Base));

            auto potential_field_generator =
                std::make_shared<phri::PotentialFieldGenerator>(
                    std::make_shared<phri::Vector3d>(
                        tcp_collision_sphere_offset_),
                    phri::ReferenceFrame::Base);

            auto obstacle_position = std::make_shared<phri::Pose>(
                phri::Vector3d(+6.3307e-03, -1.5126e-01, +9.9744e-01),
                phri::Vector3d(0., 0., 0.));
            auto obstacle = std::make_shared<phri::PotentialFieldObject>(
                phri::PotentialFieldType::Repulsive,
                std::make_shared<double>(300.), // gain
                std::make_shared<double>(0.16), // threshold distance
                obstacle_position);

            potential_field_generator->add("obstacle", obstacle);

            auto separation_dist_vel_cstr =
                std::make_shared<phri::SeparationDistanceConstraint>(
                    std::make_shared<phri::VelocityConstraint>(
                        vmax_interpolator_output_),
                    max_vel_interpolator_);

            separation_dist_vel_cstr->add("operator", operator_position_tcp_);

            controller_.add("sep dist cstr", separation_dist_vel_cstr);

            controller_.add("potential field", potential_field_generator);

            std::cout << "Trajectory init done" << std::endl;
            replay_state_ = ReplayStates::SetupTrajectory;
        } break;
        case ReplayStates::SetupTrajectory: {
            if (force_control_at_next_wp_) {
                controller_.add("emergency stop",
                                phri::EmergencyStopConstraint(
                                    std::make_shared<double>(10.),
                                    std::make_shared<double>(1.)));
            }

            (*stiffness_mat_)(0, 0) = STIFFNESS;
            (*stiffness_mat_)(1, 1) = STIFFNESS;
            (*stiffness_mat_)(2, 2) = STIFFNESS;
            *robot_->controlPointTargetPose() =
                *robot_->controlPointCurrentPose();

            if (setupTrajectoryGenerator()) {
                std::cout << "Trajectory setup done" << std::endl;
                replay_state_ = ReplayStates::ReachingWaypoint;
            } else {
                std::cout << "No more waypoints" << std::endl;
                replay_state_ = ReplayStates::End;
            }
        }

        break;
        case ReplayStates::ReachingWaypoint: {
            *robot_->controlPointTargetPose() =
                *trajectory_generator_->getPositionOutput();
            double error = (robot_->controlPointTargetPose()->translation() -
                            robot_->controlPointCurrentPose()->translation())
                               .norm();
            if (trajectory_generator_->compute() and error < 0.001) {
                std::cout << "Waypoint reached" << std::endl;
                waypoints_.pop_front();
                if (waypoints_.size() > 0) {
                    if (force_control_at_next_wp_) {
                        replay_state_ = ReplayStates::ForceControlInit;
                    } else {
                        replay_state_ = ReplayStates::SetupTrajectory;
                    }
                    force_control_at_next_wp_ = not force_control_at_next_wp_;
                } else {
                    replay_state_ = ReplayStates::End;
                }
            }
        } break;
        case ReplayStates::ForceControlInit: {
            controller_.removeConstraint("emergency stop");

            auto velocity_constraint =
                std::make_shared<phri::VelocityConstraint>(
                    std::make_shared<double>(FORCE_CTRL_VLIM));
            auto acceleration_constraint =
                std::make_shared<phri::AccelerationConstraint>(
                    std::make_shared<double>(FORCE_CTRL_ALIM), SAMPLE_TIME);

            auto target_force =
                std::make_shared<phri::Vector6d>(phri::Vector6d::Zero());
            auto p_gain = std::make_shared<phri::Vector6d>(
                phri::Vector6d::Ones() * FORCE_CTRL_PGAIN);
            auto d_gain = std::make_shared<phri::Vector6d>(
                phri::Vector6d::Ones() * FORCE_CTRL_DGAIN);
            auto selection =
                std::make_shared<phri::Vector6d>(phri::Vector6d::Zero());

            target_force->z() = FORCE_TARGET;
            selection->z() = 1.;

            auto force_control = std::make_shared<phri::ForceControl>(
                target_force, SAMPLE_TIME, p_gain, d_gain, selection);

            force_control->configureFilter(SAMPLE_TIME, 0.1);

            controller_.add("force control vlim", velocity_constraint);

            controller_.add("force control alim", acceleration_constraint);

            controller_.add("force control", force_control);

            (*stiffness_mat_)(0, 0) =
                0.; // seems to be needed because of a strange behavior in vrep
            (*stiffness_mat_)(1, 1) =
                0.; // seems to be needed because of a strange behavior in vrep
            (*stiffness_mat_)(2, 2) = 0.;

            last_time_point_ = current_time;
            is_force_ok_ = false;

            std::cout << "Force control init done" << std::endl;

            replay_state_ = ReplayStates::ForceControl;
        } break;
        case ReplayStates::ForceControl:
            if (not is_force_ok_) {
                last_time_point_ = current_time;
            }

            is_force_ok_ = std::abs(robot_->controlPointExternalForce()->z() +
                                    FORCE_TARGET) < 0.1 * FORCE_TARGET;

            if (dt > FORCE_DURATION) {
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

    computeNullSpaceVelocityVector();

    return end;
}

bool StateMachine::goToInitPose() {
    switch (init_state_) {
    case InitStates::SetupTrajectory:
        setupJointTrajectoryGenerator();
        controller_.add("init joint traj vel",
                        phri::JointVelocityProxy(
                            joint_trajectory_generator_->getVelocityOutput()));
        init_state_ = InitStates::GoToInitPose;
        break;
    case InitStates::GoToInitPose:
        if (joint_trajectory_generator_->compute()) {
            controller_.removeJointVelocityGenerator("init joint traj vel");
            init_state_ = InitStates::InitPoseReached;
        }
        break;
    case InitStates::InitPoseReached:
        init_state_ = InitStates::SetupTrajectory;
        break;
    }

    return init_state_ == InitStates::InitPoseReached;
}

bool StateMachine::setupTrajectoryGenerator() {
    if (waypoints_.size() == 0)
        return false;

    auto& from = *robot_->controlPointCurrentPose();
    auto to = waypoints_.front();
    std::cout << "Going from [" << from << "] to [" << to << "]" << std::endl;

    *(start_point_.y) = from;
    *(end_point_.y) = to;

    trajectory_generator_->reset();
    trajectory_generator_->computeTimings();

    return true;
}

bool StateMachine::setupJointTrajectoryGenerator() {
    const auto& from = *robot_->jointCurrentPosition();
    phri::VectorXd to(7);
    to << 90., -45., 0., -90., 0., 45., 0.;
    to *= M_PI / 180.;

    std::cout << "Going from [" << from.transpose() << "] to ["
              << to.transpose() << "]" << std::endl;

    *(start_point_.y) = from;
    *(end_point_.y) = to;

    trajectory_generator_->computeTimings();

    return true;
}

void StateMachine::computeLaserDistanceInTCPFrame(
    std::shared_ptr<Pose> obstacle_position) {
    phri::Vector4d position_laser = phri::Vector4d::Ones();
    position_laser.x() = operator_position_laser_->x();
    position_laser.y() = operator_position_laser_->y();
    position_laser.z() = 0.;

    phri::Vector4d position_base = laser_base_transform_ * position_laser;

    const auto& tcp_base_transform = *robot_->transformationMatrix();
    phri::Matrix4d base_tcp_transform = phri::Matrix4d::Identity();
    base_tcp_transform.block<3, 3>(0, 0) =
        tcp_base_transform.block<3, 3>(0, 0).transpose();
    base_tcp_transform.block<3, 1>(0, 3) =
        -tcp_base_transform.block<3, 3>(0, 0).transpose() *
        tcp_base_transform.block<3, 1>(0, 3);

    phri::Vector4d position_tcp = base_tcp_transform * position_base;

    // The following is to remove the vertical component from the position
    phri::Vector3d position_tcp_rbase =
        tcp_base_transform.block<3, 3>(0, 0) * position_tcp.block<3, 1>(0, 0);
    position_tcp_rbase.y() = 0.;

    obstacle_position->orientation().setIdentity();
    obstacle_position->translation() =
        tcp_base_transform.block<3, 3>(0, 0).transpose() * position_tcp_rbase;
}

void StateMachine::computeNullSpaceVelocityVector() {
    if (static_cast<bool>(null_space_velocity_)) {
        phri::VectorXd grad(robot_->jointCount());
        const auto& joint_pos = *robot_->jointCurrentPosition();
        for (size_t i = 0; i < robot_->jointCount(); ++i) {
            double avg =
                0.5 * (joint_max_positions_(i) + joint_min_positions_(i));
            grad(i) =
                2. * (joint_pos(i) - avg) /
                std::pow(joint_max_positions_(i) - joint_min_positions_(i), 2);
        }
        *null_space_velocity_ = null_space_gain_ * grad;
    }
}
