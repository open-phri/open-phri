/*      File: interpolators_common.h
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
 * @file interpolators_common.h
 * @author Benjamin Navarro
 * @brief Common definitions used by the interpolators/trajectory generators
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/definitions.h>
#include <iostream>
#include <vector>
#include <functional>
#include <tuple>

#include <physical_quantities/vector/position.h>
#include <physical_quantities/vector/velocity.h>
#include <physical_quantities/vector/acceleration.h>
#include <physical_quantities/spatial/position.h>
#include <physical_quantities/spatial/velocity.h>
#include <physical_quantities/spatial/acceleration.h>

namespace phri {

template <typename ValueT, typename FirstDerivative = ValueT,
          typename SecondDerivative = ValueT>
class TrajectoryGenerator;
using JointTrajectoryGenerator =
    TrajectoryGenerator<vector::dyn::Position, vector::dyn::Velocity,
                        vector::dyn::Acceleration>;

/** @brief Description of a point used by the TrajectoryGenerator.
 *  @details A TrajectoryPoint is described by a 2D point (x,y) and its first
 * and second derivatives
 */
template <typename ValueT, typename FirstDerivative = ValueT,
          typename SecondDerivative = ValueT>
struct TrajectoryPoint {
    TrajectoryPoint(const std::shared_ptr<ValueT>& y,
                    const std::shared_ptr<FirstDerivative>& dy,
                    const std::shared_ptr<SecondDerivative>& d2y)
        : y(y), dy(dy), d2y(d2y) {
        createRefs();
    }

    TrajectoryPoint(const ValueT& y, const FirstDerivative& dy,
                    const SecondDerivative& d2y)
        : y(std::make_shared<ValueT>(y)),
          dy(std::make_shared<FirstDerivative>(dy)),
          d2y(std::make_shared<SecondDerivative>(d2y)) {
        createRefs();
    }

    TrajectoryPoint()
        : y(std::make_shared<ValueT>()),
          dy(std::make_shared<FirstDerivative>()),
          d2y(std::make_shared<SecondDerivative>()) {
        createRefs();

        for (size_t i = 0; i < size(); ++i) {
            double& y = yrefs_[i];
            double& dy = dyrefs_[i];
            double& d2y = d2yrefs_[i];
            y = 0.;
            dy = 0.;
            d2y = 0.;
        }
    }

    TrajectoryPoint(size_t size) : TrajectoryPoint{} {
        resize(size);
    }

    template <typename U = ValueT>
    TrajectoryPoint(
        size_t size,
        typename std::enable_if_t<std::is_same_v<U, std::vector<double>> or
                                  std::is_base_of_v<Eigen::VectorXd, U>>* = 0)
        : TrajectoryPoint() {
        resize(size);
    }

    template <typename U = ValueT>
    void resize(
        size_t size,
        typename std::enable_if_t<std::is_same_v<U, std::vector<double>> or
                                  std::is_base_of_v<Eigen::VectorXd, U>>* = 0) {
        y->resize(size);
        dy->resize(size);
        d2y->resize(size);

        createRefs();

        for (size_t i = 0; i < this->size(); ++i) {
            double& y = yrefs_[i];
            double& dy = dyrefs_[i];
            double& d2y = d2yrefs_[i];
            y = 0.;
            dy = 0.;
            d2y = 0.;
        }
    }

    void print() const {
        std::cout << "[";
        std::cout << "(";
        for (auto pt : yrefs_) {
            std::cout << pt << " ";
        }
        std::cout << ")";
        std::cout << "(";
        for (auto pt : dyrefs_) {
            std::cout << pt << " ";
        }
        std::cout << ")";
        std::cout << "(";
        for (auto pt : d2yrefs_) {
            std::cout << pt << " ";
        }
        std::cout << ")";
        std::cout << "]";
    }

    size_t size() const {
        return size_;
    }

    std::tuple<double&, double&, double&> operator[](size_t n) {
        assert(n < size_);
        return std::make_tuple(yrefs_[n], dyrefs_[n], d2yrefs_[n]);
    }

    std::shared_ptr<ValueT> y;             // value
    std::shared_ptr<FirstDerivative> dy;   // first derivative
    std::shared_ptr<SecondDerivative> d2y; // second derivative
private:
    friend class TrajectoryGenerator<ValueT, FirstDerivative, SecondDerivative>;

    size_t size_;
    std::vector<std::reference_wrapper<double>> yrefs_;
    std::vector<std::reference_wrapper<double>> dyrefs_;
    std::vector<std::reference_wrapper<double>> d2yrefs_;

    template <typename U = ValueT>
    void createRefs(typename std::enable_if_t<std::is_same_v<U, double>>* = 0) {
        size_ = 1;
        yrefs_.clear();
        dyrefs_.clear();
        d2yrefs_.clear();
        yrefs_.push_back(std::ref(*y));
        dyrefs_.push_back(std::ref(*dy));
        d2yrefs_.push_back(std::ref(*d2y));
    }

    template <typename U = ValueT>
    void
    createRefs(typename std::enable_if_t<not std::is_same_v<U, double>>* = 0) {
        assert(y->size() == dy->size() and y->size() == d2y->size());
        size_ = y->size();
        yrefs_.clear();
        dyrefs_.clear();
        d2yrefs_.clear();
        auto& yvec = *y;
        auto& dyvec = *dy;
        auto& d2yvec = *d2y;
        for (size_t i = 0; i < static_cast<size_t>(yvec.size()); ++i) {
            yrefs_.push_back(std::ref(yvec[i]));
            dyrefs_.push_back(std::ref(dyvec[i]));
            d2yrefs_.push_back(std::ref(d2yvec[i]));
        }
    }
};

template <>
struct TrajectoryPoint<spatial::Position, spatial::Velocity,
                       spatial::Acceleration> {
    TrajectoryPoint(const std::shared_ptr<spatial::Position>& y,
                    const std::shared_ptr<spatial::Velocity>& dy,
                    const std::shared_ptr<spatial::Acceleration>& d2y)
        : y(y), dy(dy), d2y(d2y) {
    }

    TrajectoryPoint(const spatial::Position& y, const spatial::Velocity& dy,
                    const spatial::Acceleration& d2y)
        : y(std::make_shared<spatial::Position>(y)),
          dy(std::make_shared<spatial::Velocity>(dy)),
          d2y(std::make_shared<spatial::Acceleration>(d2y)) {
    }

    TrajectoryPoint(spatial::Frame frame)
        : y(std::make_shared<spatial::Position>(frame)),
          dy(std::make_shared<spatial::Velocity>(frame)),
          d2y(std::make_shared<spatial::Acceleration>(frame)) {
    }

    void print() const {
        Eigen::Quaterniond q = y->orientation().asQuaternion();
        std::cout << "[";
        std::cout << "(";
        std::cout << y->linear().transpose() << "; ";
        std::cout << q.w() << " + ";
        std::cout << q.x() << "i + ";
        std::cout << q.y() << "j + ";
        std::cout << q.z() << "k";
        std::cout << ")";
        std::cout << "(";
        std::cout << static_cast<Eigen::Vector6d>(*dy).transpose();
        std::cout << ")";
        std::cout << "(";
        std::cout << static_cast<Eigen::Vector6d>(*d2y).transpose();
        std::cout << ")";
        std::cout << "]";
    }

    // std::tuple<double&, double&, double&> operator[] (size_t n) {
    //  assert(n < size_);
    //  return std::make_tuple(yrefs_[n], dyrefs_[n], d2yrefs_[n]);
    // }

    std::shared_ptr<spatial::Position> y;       // value
    std::shared_ptr<spatial::Velocity> dy;      // first derivative
    std::shared_ptr<spatial::Acceleration> d2y; // second derivative
};

} // namespace phri
