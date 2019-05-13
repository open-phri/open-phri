/*      File: task_space_data_types.h
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
 * @file task_space_data_types.h
 * @author Benjamin Navarro
 * @brief Usefull definitions being used in OpenPHRI
 * @date September 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/type_aliases.h>
#include <OpenPHRI/utilities/exceptions.h>
#include <map>
#include <vector>
#include <utility>
#include <algorithm>

namespace phri {

/**
//! \brief Specify an object's reference frame
enum class ReferenceFrame {
    //! \brief Frame attached to the tool control point
    TCP,
    //! \brief Robot's base frame
    Base,
    //! \brief Frame fixed relative to the environment
    World
};
**/

enum class ReferenceFrame : size_t {};

template <typename T> class TaskSpaceData;

class FrameAdapter {
public:
    template <typename T>
    using is_task_space_data_t =
        typename std::enable_if<std::is_base_of<TaskSpaceData<T>, T>::value,
                                T>::type;

    static void setTransform(ReferenceFrame from,
                             const AffineTransform& transform,
                             ReferenceFrame to) {
        transforms()[{from, to}] = transform;
    };

    template <typename T>
    static is_task_space_data_t<T> transform(const T& data, ReferenceFrame to) {
        if (data.frame() == to) {
            return data;
        } else {
            return static_cast<T>(getTransform(data.frame(), to) * data);
        }
    }

    static AffineTransform getTransform(ReferenceFrame from,
                                        ReferenceFrame to) {
        try {
            return transforms().at({from, to});
        } catch (const std::out_of_range&) {
            try {
                return transforms().at({to, from}).inverse();
            } catch (const std::out_of_range&) {
                throw std::out_of_range(
                    OPEN_PHRI_ERROR("There is no relationship between the "
                                    "origin and target frames"));
            }
        }
    }

    static ReferenceFrame frame(const std::string& name) {
        auto existing = findFrame(name);
        if (existing == frames().end()) {
            frames().push_back(name);
            return ReferenceFrame(frames().size() - 1);
        } else {
            return getFrame(existing);
        }
    }

    static ReferenceFrame world() {
        return ReferenceFrame(0);
    }

private:
    using FromTo = std::pair<ReferenceFrame, ReferenceFrame>;
    static std::map<FromTo, AffineTransform>& transforms() {
        static std::map<FromTo, AffineTransform> transforms_;
        return transforms_;
    }

    static std::vector<std::string>& frames() {
        static std::vector<std::string> frames_ = {"world"};
        return frames_;
    }

    static decltype(frames().begin()) findFrame(const std::string& name) {
        return std::find(frames().begin(), frames().end(), name);
    }

    static ReferenceFrame getFrame(decltype(frames().begin()) position) {
        return ReferenceFrame(std::distance(frames().begin(), position));
    }
};

template <typename Derived> class TaskSpaceData {
public:
    TaskSpaceData() : frame_(FrameAdapter::world()) {
    }

    TaskSpaceData(ReferenceFrame frame) : frame_(frame) {
    }

    ReferenceFrame& frame() {
        return frame_;
    }

    ReferenceFrame frame() const {
        return frame_;
    }

    Derived transform(ReferenceFrame to) const {
        return FrameAdapter::transform(static_cast<const Derived&>(*this), to);
    }

private:
    ReferenceFrame frame_;
};

class Twist;
class Acceleration;
class Pose : public TaskSpaceData<Pose> {
public:
    explicit Pose(ReferenceFrame frame = FrameAdapter::world());

    Pose(phri::Vector3d translation, Eigen::Quaterniond orientation,
         ReferenceFrame frame = FrameAdapter::world());

    Pose(phri::Vector3d translation, phri::Vector3d euler_angles,
         ReferenceFrame frame = FrameAdapter::world());

    explicit Pose(phri::AffineTransform transformation,
                  ReferenceFrame frame = FrameAdapter::world());

    const phri::Vector3d& translation() const;
    const Eigen::Quaterniond& orientation() const;
    phri::Vector3d& translation();
    Eigen::Quaterniond& orientation();

    phri::Vector6d getErrorWith(const Pose& other) const;
    Pose& integrate(const Twist& twist, double sample_time);

    operator phri::AffineTransform() const;
    explicit operator phri::Vector6d() const;
    Pose& operator=(const phri::Vector6d& pos_euler);

    friend std ::ostream& operator<<(std::ostream& out, const Pose& pose) {
        out << pose.translation().x() << "x + " << pose.translation().y()
            << "y + " << pose.translation().z() << "z; ";
        out << pose.orientation().w() << " + " << pose.orientation().x()
            << "i + " << pose.orientation().y() << "j + "
            << pose.orientation().z() << "k";
        return (out);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    phri::Vector3d translation_;
    Eigen::Quaterniond orientation_;
};

class Twist : public TaskSpaceData<Twist> {
public:
    Twist(ReferenceFrame frame = FrameAdapter::world());

    Twist(const phri::Vector3d& translation, const phri::Vector3d& rotation,
          ReferenceFrame frame = FrameAdapter::world());

    explicit Twist(const phri::Vector6d& twist,
                   ReferenceFrame frame = FrameAdapter::world());

    Eigen::Ref<const phri::Vector3d> translation() const;
    Eigen::Ref<const phri::Vector3d> rotation() const;
    Eigen::Ref<phri::Vector3d> translation();
    Eigen::Ref<phri::Vector3d> rotation();

    Twist& integrate(const Acceleration& acceleration, double sample_time);

    Eigen::Ref<const phri::Vector6d> vector() const;
    Eigen::Ref<phri::Vector6d> vector();

    operator const phri::Vector6d&() const;
    operator phri::Vector6d&();

    double* data();
    const double* data() const;

    Twist& operator=(const phri::Vector6d& twist);
    Twist operator-(const phri::Twist& other) const;
    Twist operator+(const phri::Twist& other) const;
    Twist operator*(double scalar) const;
    Twist operator/(double scalar) const;
    Twist& operator-=(const phri::Twist& other);
    Twist& operator+=(const phri::Twist& other);
    Twist& operator*=(double scalar);
    Twist& operator/=(double scalar);

    friend std ::ostream& operator<<(std::ostream& out, const Twist& twist) {
        out << static_cast<phri::Vector6d>(twist).transpose();
        return (out);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    phri::Vector6d twist_;
};

Twist operator*(const AffineTransform& transform, const Twist& twist);

class Acceleration : public TaskSpaceData<Acceleration> {
public:
    Acceleration(ReferenceFrame frame = FrameAdapter::world());

    Acceleration(phri::Vector3d translation, phri::Vector3d rotation,
                 ReferenceFrame frame = FrameAdapter::world());

    explicit Acceleration(phri::Vector6d acceleration,
                          ReferenceFrame frame = FrameAdapter::world());

    Eigen::Ref<const phri::Vector3d> translation() const;
    Eigen::Ref<const phri::Vector3d> rotation() const;
    Eigen::Ref<phri::Vector3d> translation();
    Eigen::Ref<phri::Vector3d> rotation();

    Eigen::Ref<const phri::Vector6d> vector() const;
    Eigen::Ref<phri::Vector6d> vector();

    operator const phri::Vector6d&() const;
    operator phri::Vector6d&();

    double* data();
    const double* data() const;

    Acceleration& operator=(const phri::Vector6d& acceleration);
    Acceleration operator-(const phri::Acceleration& other) const;
    Acceleration operator+(const phri::Acceleration& other) const;
    Acceleration operator*(double scalar) const;
    Acceleration operator/(double scalar) const;
    Acceleration& operator-=(const phri::Acceleration& other);
    Acceleration& operator+=(const phri::Acceleration& other);
    Acceleration& operator*=(double scalar);
    Acceleration& operator/=(double scalar);

    friend std ::ostream& operator<<(std::ostream& out,
                                     const Acceleration& acceleration) {
        out << static_cast<phri::Vector6d>(acceleration).transpose();
        return (out);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    phri::Vector6d acceleration_;
};

Acceleration operator*(const AffineTransform& transform,
                       const Acceleration& twist);

class Wrench : public TaskSpaceData<Wrench> {
public:
    Wrench(ReferenceFrame frame = FrameAdapter::world());

    Wrench(phri::Vector3d force, phri::Vector3d torque,
           ReferenceFrame frame = FrameAdapter::world());

    explicit Wrench(phri::Vector6d wrench,
                    ReferenceFrame frame = FrameAdapter::world());

    Eigen::Ref<const phri::Vector3d> force() const;
    Eigen::Ref<const phri::Vector3d> torque() const;
    Eigen::Ref<phri::Vector3d> force();
    Eigen::Ref<phri::Vector3d> torque();

    Eigen::Ref<const phri::Vector6d> vector() const;
    Eigen::Ref<phri::Vector6d> vector();

    operator const phri::Vector6d&() const;
    operator phri::Vector6d&();

    double* data();
    const double* data() const;

    Wrench& operator=(const phri::Vector6d& wrench);
    Wrench operator-(const phri::Wrench& other) const;
    Wrench operator+(const phri::Wrench& other) const;
    Wrench operator*(double scalar) const;
    Wrench operator/(double scalar) const;
    Wrench& operator-=(const phri::Wrench& other);
    Wrench& operator+=(const phri::Wrench& other);
    Wrench& operator*=(double scalar);
    Wrench& operator/=(double scalar);

    friend std ::ostream& operator<<(std::ostream& out, const Wrench& wrench) {
        out << static_cast<phri::Vector6d>(wrench).transpose();
        return (out);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    phri::Vector6d wrench_;
};

Wrench operator*(const AffineTransform& transform, const Wrench& twist);

} // namespace phri