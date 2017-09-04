
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
