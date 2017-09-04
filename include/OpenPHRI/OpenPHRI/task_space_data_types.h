/*      File: task_space_data_types.h
*       This file is part of the program open-phri
*       Program description : OpenPHRI: a generic framework to easily and safely control robots in interactions with humans
*       Copyright (C) 2017 -  Benjamin Navarro (LIRMM). All Right reserved.
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
*       You should have received a copy of the GNU Lesser General Public License version 3 and the
*       General Public License version 3 along with this program.
*       If not, see <http://www.gnu.org/licenses/>.
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

namespace phri {

class Twist;
class Acceleration;

class Pose {
public:
	Pose();
	Pose(std::shared_ptr<phri::Vector3d> translation, std::shared_ptr<Eigen::Quaterniond> orientation);
	Pose(phri::Vector3d translation, Eigen::Quaterniond orientation);
	Pose(phri::Vector3d translation, phri::Vector3d euler_angles);
	explicit Pose(phri::AffineTransform transformation);

	const phri::Vector3d&                       translation() const;
	const Eigen::Quaterniond&                   orientation() const;
	phri::Vector3d&                             translation();
	Eigen::Quaterniond&                         orientation();

	std::shared_ptr<const phri::Vector3d>       translationPtr() const;
	std::shared_ptr<const Eigen::Quaterniond>   orientationPtr() const;
	std::shared_ptr<phri::Vector3d>             translationPtr();
	std::shared_ptr<Eigen::Quaterniond>         orientationPtr();

	phri::Vector6d getErrorWith(const Pose& other) const;
	Pose& integrate(const Twist& twist, double sample_time);

	operator phri::AffineTransform() const;

private:
	std::shared_ptr<phri::Vector3d> translation_;
	std::shared_ptr<Eigen::Quaterniond> orientation_;
};

class Twist {
public:
	Twist();
	Twist(std::shared_ptr<phri::Vector3d> translation, std::shared_ptr<phri::Vector3d> rotation);
	Twist(phri::Vector3d translation, phri::Vector3d rotation);
	explicit Twist(phri::Vector6d twist);

	const phri::Vector3d&                   translation() const;
	const phri::Vector3d&                   rotation() const;
	phri::Vector3d&                         translation();
	phri::Vector3d&                         rotation();

	std::shared_ptr<const phri::Vector3d>   translationPtr() const;
	std::shared_ptr<const phri::Vector3d>   rotationPtr() const;
	std::shared_ptr<phri::Vector3d>         translationPtr();
	std::shared_ptr<phri::Vector3d>         rotationPtr();

	Twist& integrate(const Acceleration& acceleration, double sample_time);

	operator phri::Vector6d() const;

private:
	std::shared_ptr<phri::Vector3d> translation_;
	std::shared_ptr<phri::Vector3d> rotation_;
};

class Acceleration {
public:
	Acceleration();
	Acceleration(std::shared_ptr<phri::Vector3d> translation, std::shared_ptr<phri::Vector3d> rotation);
	Acceleration(phri::Vector3d translation, phri::Vector3d rotation);
	explicit Acceleration(phri::Vector6d acceleration);

	const phri::Vector3d&                   translation() const;
	const phri::Vector3d&                   rotation() const;
	phri::Vector3d&                         translation();
	phri::Vector3d&                         rotation();

	std::shared_ptr<const phri::Vector3d>   translationPtr() const;
	std::shared_ptr<const phri::Vector3d>   rotationPtr() const;
	std::shared_ptr<phri::Vector3d>         translationPtr();
	std::shared_ptr<phri::Vector3d>         rotationPtr();

	operator phri::Vector6d() const;

private:
	std::shared_ptr<phri::Vector3d> translation_;
	std::shared_ptr<phri::Vector3d> rotation_;
};

} // namespace phri
