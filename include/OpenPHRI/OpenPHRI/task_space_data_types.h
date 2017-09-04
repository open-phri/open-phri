
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

class Pose {
public:
	Pose();
	Pose(std::shared_ptr<phri::Vector3d> translation, std::shared_ptr<Eigen::Quaterniond> orientation);
	Pose(phri::Vector3d translation, Eigen::Quaterniond orientation);
	Pose(phri::Vector3d translation, phri::Vector3d euler_angles);
	explicit Pose(phri::AffineTransform transformation);

	std::shared_ptr<const phri::Vector3d> translation() const;
	std::shared_ptr<const Eigen::Quaterniond> orientation() const;
	std::shared_ptr<phri::Vector3d> translation();
	std::shared_ptr<Eigen::Quaterniond> orientation();

	phri::Vector6d getErrorWith(const Pose& other) const;

	operator phri::AffineTransform() const;

private:
	std::shared_ptr<phri::Vector3d> translation_;
	std::shared_ptr<Eigen::Quaterniond> orientation_;
};

class Twist {
public:
	Twist();
	Twist(std::shared_ptr<phri::Vector3d> translation, std::shared_ptr<Eigen::Quaterniond> rotation);
	Twist(phri::Vector3d translation, phri::Vector3d rotation);
	explicit Twist(phri::Vector6d twist);

	std::shared_ptr<const phri::Vector3d> translation() const;
	std::shared_ptr<const Eigen::Quaterniond> rotation() const;
	std::shared_ptr<phri::Vector3d> translation();
	std::shared_ptr<Eigen::Quaterniond> rotation();

	operator phri::Vector6d() const;

private:
	std::shared_ptr<phri::Vector3d> translation_;
	std::shared_ptr<Eigen::Quaterniond> orientation_;
};

class Acceleration {
public:
	Acceleration();
	Acceleration(std::shared_ptr<phri::Vector3d> translation, std::shared_ptr<Eigen::Quaterniond> rotation);
	Acceleration(phri::Vector3d translation, phri::Vector3d rotation);
	explicit Acceleration(phri::Vector6d acceleration);

	std::shared_ptr<const phri::Vector3d> translation() const;
	std::shared_ptr<const Eigen::Quaterniond> rotation() const;
	std::shared_ptr<phri::Vector3d> translation();
	std::shared_ptr<Eigen::Quaterniond> rotation();

	operator phri::Vector6d() const;

private:
	std::shared_ptr<phri::Vector3d> translation_;
	std::shared_ptr<Eigen::Quaterniond> orientation_;
};

} // namespace phri
