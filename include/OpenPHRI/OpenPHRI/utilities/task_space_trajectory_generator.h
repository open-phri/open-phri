/*      File: task_space_trajectory_generator.h
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

/**
 * @file task_space_trajectory_generator.h
 * @author Benjamin Navarro
 * @brief Definition of the TrajectoryPoint struct and the TrajectoryGenerator
 * class
 * @date September 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/utilities/trajectory_generator.h>
#include <list>

namespace phri {

template <>
class TrajectoryGenerator<spatial::Position>
    : public TrajectoryGenerator<Eigen::Vector6d> {
    using super = TrajectoryGenerator<Eigen::Vector6d>;

public:
    TrajectoryGenerator(
        const TrajectoryPoint<spatial::Position>& start, double sample_time,
        TrajectorySynchronization sync =
            TrajectorySynchronization::NoSynchronization,
        TrajectoryOutputType output_type = TrajectoryOutputType::All)
        : TrajectoryGenerator<Eigen::Vector6d>(sample_time, sync, output_type),
          start_pose_(start) {
        super::create(start_vec_);
        pose_output_ = std::make_shared<spatial::Position>(*start.y);
        twist_output_ = std::make_shared<spatial::Velocity>(*start.dy);
        task_space_acceleration_output_ =
            std::make_shared<spatial::Acceleration>(*start.d2y);
    }

    void addPathTo(const TrajectoryPoint<spatial::Position>& to,
                   const spatial::Velocity& max_velocity,
                   const spatial::Acceleration& max_acceleration) {
        waypoints_.emplace_back(to, max_velocity, max_acceleration);
    }

    std::shared_ptr<const spatial::Position> getPoseOutput() {
        return pose_output_;
    }

    std::shared_ptr<const spatial::Position> getPositionOutput() {
        return getPoseOutput();
    }

    std::shared_ptr<const spatial::Velocity> getTwistOutput() {
        return twist_output_;
    }

    std::shared_ptr<const spatial::Velocity> getVelocityOutput() {
        return getTwistOutput();
    }

    std::shared_ptr<const spatial::Acceleration> getAccelerationOutput() {
        return task_space_acceleration_output_;
    }

    virtual bool computeTimings(double v_eps = 1e-6,
                                double a_eps = 1e-6) override {
        updatePoints();
        return super::computeTimings(v_eps, a_eps);
    }

    virtual bool compute() override {
        if (error_tracking_params_) {
            reference_pose_vec_->block<3, 1>(0, 0) = reference_pose_->linear();
            reference_pose_vec_->block<3, 1>(3, 0) =
                start_pose_.y->getErrorWith(*reference_pose_).tail<3>();
        }

        bool ret = super::compute();

        if (output_type_ == TrajectoryOutputType::Position or
            output_type_ == TrajectoryOutputType::All) {
            pose_output_->linear() = position_output_->block<3, 1>(0, 0);
            pose_output_->orientation() =
                start_pose_.y->orientation().asQuaternion().integrate(
                    position_output_->block<3, 1>(3, 0));
        }
        if (output_type_ == TrajectoryOutputType::Velocity or
            output_type_ == TrajectoryOutputType::All) {
            twist_output_->value() = *velocity_output_;
        }
        if (output_type_ == TrajectoryOutputType::Acceleration or
            output_type_ == TrajectoryOutputType::All) {
            task_space_acceleration_output_->value() = *acceleration_output_;
        }

        return ret;
    }

    virtual void removeAllPoints() override {
        waypoints_.clear();
        super::removeAllPoints();
    }

    void enableErrorTracking(std::shared_ptr<const spatial::Position> reference,
                             const Eigen::Vector6d& threshold,
                             bool recompute_when_resumed,
                             double hysteresis_threshold = 0.1) {
        reference_pose_ = reference;
        reference_pose_vec_ = std::make_shared<Eigen::Vector6d>();
        error_tracking_params_.reference = reference_pose_vec_;
        error_tracking_params_.threshold = threshold;
        error_tracking_params_.hysteresis_threshold = hysteresis_threshold;
        error_tracking_params_.recompute_when_resumed = recompute_when_resumed;
        for (size_t i = 0; i < 6; ++i) {
            error_tracking_params_.reference_refs.push_back(
                std::cref((*error_tracking_params_.reference)[i]));
            error_tracking_params_.threshold_refs.push_back(
                std::cref(error_tracking_params_.threshold[i]));
        }
    }

    void enableErrorTracking(const spatial::Position* reference,
                             const Eigen::Vector6d& threshold,
                             bool recompute_when_resumed,
                             double hysteresis_threshold = 0.1) {
        enableErrorTracking(std::shared_ptr<const spatial::Position>(
                                reference, [](const spatial::Position* p) {}),
                            threshold, recompute_when_resumed);
    }

private:
    struct Point {
        Point(TrajectoryPoint<spatial::Position> pose,
              spatial::Velocity max_velocity,
              spatial::Acceleration max_acceleration)
            : pose(pose),
              max_velocity(max_velocity),
              max_acceleration(max_acceleration) {
        }
        TrajectoryPoint<spatial::Position> pose;
        spatial::Velocity max_velocity;
        spatial::Acceleration max_acceleration;
    };

    void updatePoints() {
        *pose_output_ = *start_pose_.y;
        *twist_output_ = *start_pose_.dy;
        *task_space_acceleration_output_ = *start_pose_.d2y;

        super::removeAllPoints();

        auto wp = waypoints_.begin();
        if (wp != waypoints_.end()) {
            start_vec_.y->block<3, 1>(0, 0) = start_pose_.y->linear();
            start_vec_.y->block<3, 1>(3, 0).setZero();
            *start_vec_.dy = static_cast<Eigen::Vector6d>(*start_pose_.dy);
            *start_vec_.d2y = static_cast<Eigen::Vector6d>(*start_pose_.d2y);

            Point prev(start_pose_, spatial::Velocity(start_pose_.y->frame()),
                       spatial::Acceleration(start_pose_.y->frame()));
            while (wp != waypoints_.end()) {
                TrajectoryPoint<Eigen::Vector6d> to;
                to.y->block<3, 1>(0, 0) = wp->pose.y->linear();
                to.y->block<3, 1>(3, 0) =
                    prev.pose.y->getErrorWith(*wp->pose.y).tail<3>();
                *to.dy = *wp->pose.dy;
                *to.d2y = *wp->pose.d2y;
                super::addPathTo(
                    to, static_cast<Eigen::Vector6d>(wp->max_velocity),
                    static_cast<Eigen::Vector6d>(wp->max_acceleration));
                prev = *wp;
                ++wp;
            }
        }
    }

    TrajectoryPoint<Eigen::Vector6d> start_vec_;
    TrajectoryPoint<spatial::Position> start_pose_;
    std::shared_ptr<const spatial::Position> reference_pose_;
    std::shared_ptr<Eigen::Vector6d> reference_pose_vec_;
    std::list<Point> waypoints_;
    std::shared_ptr<spatial::Position> pose_output_;
    std::shared_ptr<spatial::Velocity> twist_output_;
    std::shared_ptr<spatial::Acceleration> task_space_acceleration_output_;
};

using TaskSpaceTrajectoryGenerator = TrajectoryGenerator<spatial::Position>;

extern template class TrajectoryGenerator<spatial::Position>;

} // namespace phri
