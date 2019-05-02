/*      File: state_machine.h
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

#pragma once

#include <OpenPHRI/OpenPHRI.h>
#include <chrono>
#include <list>

class StateMachine {
public:
    enum class InitStates { SetupTrajectory, GoToInitPose, InitPoseReached };

    enum class TeachStates { Init, WaitForMotion, Motion, End };

    enum class ReplayStates {
        Init,
        SetupTrajectory,
        ReachingWaypoint,
        ForceControlInit,
        ForceControl,
        ForceControlEnd,
        End
    };

    StateMachine(phri::RobotPtr robot, phri::SafetyController& controller,
                 std::shared_ptr<phri::LaserScannerDetector> laser_detector,
                 bool skip_teaching = false);
    ~StateMachine() = default;

    // Read init position, set parameters
    void init();

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
    void computeLaserDistanceInTCPFrame(phri::PosePtr obstacle_position);
    void computeNullSpaceVelocityVector();

    phri::RobotPtr robot_;
    phri::SafetyController& controller_;
    std::shared_ptr<phri::LaserScannerDetector> laser_detector_;

    InitStates init_state_;
    TeachStates teach_state_;
    ReplayStates replay_state_;
    std::list<phri::Pose> waypoints_;
    phri::TrajectoryPoint<phri::VectorXd> joint_start_point_;
    phri::TrajectoryPoint<phri::VectorXd> joint_end_point_;
    phri::TrajectoryPoint<phri::Pose> start_point_;
    phri::TrajectoryPoint<phri::Pose> end_point_;
    std::shared_ptr<phri::TaskSpaceTrajectoryGenerator> trajectory_generator_;
    std::shared_ptr<phri::TrajectoryGenerator<phri::VectorXd>>
        joint_trajectory_generator_;
    std::shared_ptr<phri::LinearInterpolator> max_vel_interpolator_;
    std::shared_ptr<phri::Pose> init_position_;
    std::shared_ptr<phri::Twist> traj_vel_;
    std::shared_ptr<phri::Matrix6d> stiffness_mat_;
    std::shared_ptr<const double> vmax_interpolator_output_;
    std::shared_ptr<const phri::Vector2d> operator_position_laser_;
    std::shared_ptr<phri::Pose> operator_position_tcp_;
    std::shared_ptr<phri::Pose> tcp_collision_sphere_center_;
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
