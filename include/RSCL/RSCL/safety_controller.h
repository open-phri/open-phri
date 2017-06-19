/*
 *  Copyright (C) 2017 Benjamin Navarro <contact@bnavarro.info>
 *
 *  This file is part of RSCL <https://gite.lirmm.fr/navarro/RSCL>.
 *
 *  RSCL is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  RSCL is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with RSCL.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file safety_controller.h
 * @author Benjamin Navarro
 * @brief Definition of the SafetyController class
 * @date April 2017
 * @ingroup RSCL
 */

#pragma once

#include <map>

#include <RSCL/definitions.h>
#include <RSCL/robot.h>
#include <RSCL/fwd_decl.h>
#include <RSCL/utilities/object_collection.hpp>

namespace RSCL {

/** @brief The controller class. Deals with all the generators (velocity & force) and the constraints to compute the TCP velocity command.
 *  @details Provides mechanisms to add/get/remove constraints and generators (can be done at any time) and to get the velocity command as well as intermediate computation values (mainly used by constraints).
 */
class SafetyController {
public:
	/**
	 * @brief Construct a safety controller with a given robot.
	 * @param robot The robot to work with.
	 */
	SafetyController(RobotPtr robot);

	~SafetyController() = default;

	/**
	 * @brief Set the verbosity on or off. If on, error messages can be printed when calling the add/get/remove methods.
	 * @param on True to enable verbosity, false to disable it.
	 */
	void setVerbose(bool on);

	/**
	 * @brief Add a new constraint to the controller.
	 * @param name The name given to the constraint. Must be unique. Used to latter get/remove the constraint.
	 * @param constraint A shared pointer to the constraint to add.
	 * @param force [optional] If true, any constraint with the same name will be overriden.
	 * @return true if the constraint has successfully been added to the controller, false otherwise.
	 */
	bool addConstraint(const std::string& name, std::shared_ptr<Constraint> constraint, bool force = false);

	/**
	 * @brief Add a new force generator to the controller.
	 * @param name The name given to the force generator. Must be unique. Used to latter get/remove the force generator.
	 * @param generator A shared pointer to the force generator to add.
	 * @param force [optional] If true, any force generator with the same name will be overriden.
	 * @return true if the force generator has successfully been added to the controller, false otherwise.
	 */
	bool addForceGenerator(const std::string& name, std::shared_ptr<ForceGenerator> generator, bool force = false);

	/**
	 * @brief Add a new torque generator to the controller.
	 * @param name The name given to the torque generator. Must be unique. Used to latter get/remove the torque generator.
	 * @param generator A shared pointer to the torque generator to add.
	 * @param force [optional] If true, any force generator with the same name will be overriden.
	 * @return true if the torque generator has successfully been added to the controller, false otherwise.
	 */
	bool addTorqueGenerator(const std::string& name, std::shared_ptr<TorqueGenerator> generator, bool force = false);

	/**
	 * @brief Add a new velocity generator to the controller.
	 * @param name The name given to the velocity generator. Must be unique. Used to latter get/remove the velocity generator.
	 * @param generator A shared pointer to the velocity generator to add.
	 * @param force [optional] If true, any velocity generator with the same name will be overriden.
	 * @return true if the velocity generator has successfully been added to the controller, false otherwise.
	 */
	bool addVelocityGenerator(const std::string& name, std::shared_ptr<VelocityGenerator> generator, bool force = false);

	/**
	 * @brief Add a new joint velocity generator to the controller.
	 * @param name The name given to the joint velocity generator. Must be unique. Used to latter get/remove the joint velocity generator.
	 * @param generator A shared pointer to the joint velocity generator to add.
	 * @param force [optional] If true, any joint velocity generator with the same name will be overriden.
	 * @return true if the joint velocity generator has successfully been added to the controller, false otherwise.
	 */
	bool addJointVelocityGenerator(const std::string& name, std::shared_ptr<JointVelocityGenerator> generator, bool force = false);


	/**
	 * @brief Shortcut for the SafetyController::addConstraint method.
	 */
	template<typename T>
	typename std::enable_if<std::is_base_of<Constraint, T>::value, bool>::type
	add(const std::string& name, std::shared_ptr<T> obj, bool force = false) {
		return addConstraint(name, obj, force);
	}

	/**
	 * @brief Shortcut for the SafetyController::addConstraint method.
	 */
	template<typename T>
	typename std::enable_if<std::is_base_of<Constraint, T>::value, bool>::type
	add(const std::string& name, T&& obj, bool force = false) {
		return addConstraint(name, std::make_shared<T>(obj), force);
	}

	/**
	 * @brief Shortcut for the SafetyController::addForceGenerator method.
	 */
	template<typename T>
	typename std::enable_if<std::is_base_of<ForceGenerator, T>::value, bool>::type
	add(const std::string& name, std::shared_ptr<T> obj, bool force = false) {
		return addForceGenerator(name, obj, force);
	}

	/**
	 * @brief Shortcut for the SafetyController::addForceGenerator method.
	 */
	template<typename T>
	typename std::enable_if<std::is_base_of<ForceGenerator, T>::value, bool>::type
	add(const std::string& name, T&& obj, bool force = false) {
		return addForceGenerator(name, std::make_shared<T>(obj), force);
	}

	/**
	 * @brief Shortcut for the SafetyController::addTorqueGenerator method.
	 */
	template<typename T>
	typename std::enable_if<std::is_base_of<TorqueGenerator, T>::value, bool>::type
	add(const std::string& name, std::shared_ptr<T> obj, bool force = false) {
		return addTorqueGenerator(name, obj, force);
	}

	/**
	 * @brief Shortcut for the SafetyController::addTorqueGenerator method.
	 */
	template<typename T>
	typename std::enable_if<std::is_base_of<TorqueGenerator, T>::value, bool>::type
	add(const std::string& name, T&& obj, bool force = false) {
		return addTorqueGenerator(name, std::make_shared<T>(obj), force);
	}

	/**
	 * @brief Shortcut for the SafetyController::addVelocityGenerator method.
	 */
	template<typename T>
	typename std::enable_if<std::is_base_of<VelocityGenerator, T>::value, bool>::type
	add(const std::string& name, std::shared_ptr<T> obj, bool force = false) {
		return addVelocityGenerator(name, obj, force);
	}

	/**
	 * @brief Shortcut for the SafetyController::addVelocityGenerator method.
	 */
	template<typename T>
	typename std::enable_if<std::is_base_of<VelocityGenerator, T>::value, bool>::type
	add(const std::string& name, T&& obj, bool force = false) {
		return addVelocityGenerator(name, std::make_shared<T>(obj), force);
	}

	/**
	 * @brief Shortcut for the SafetyController::addJointVelocityGenerator method.
	 */
	template<typename T>
	typename std::enable_if<std::is_base_of<JointVelocityGenerator, T>::value, bool>::type
	add(const std::string& name, std::shared_ptr<T> obj, bool force = false) {
		return addJointVelocityGenerator(name, obj, force);
	}

	/**
	 * @brief Shortcut for the SafetyController::addJointVelocityGenerator method.
	 */
	template<typename T>
	typename std::enable_if<std::is_base_of<JointVelocityGenerator, T>::value, bool>::type
	add(const std::string& name, T&& obj, bool force = false) {
		return addJointVelocityGenerator(name, std::make_shared<T>(obj), force);
	}

	/**
	 * @brief Remove a constraint from the controller.
	 * @param name The name given to the constraint to remove.
	 * @return true if the constraint has successfully been removed from the controller, false otherwise.
	 */
	bool removeConstraint(const std::string& name);

	/**
	 * @brief Remove a force generator from the controller.
	 * @param name The name given to the force generator to remove.
	 * @return true if the force generator has successfully been removed from the controller, false otherwise.
	 */
	bool removeForceGenerator(const std::string& name);

	/**
	 * @brief Remove a torque generator from the controller.
	 * @param name The name given to the torque generator to remove.
	 * @return true if the torque generator has successfully been removed from the controller, false otherwise.
	 */
	bool removeTorqueGenerator(const std::string& name);

	/**
	 * @brief Remove a velocity generator from the controller.
	 * @param name The name given to the velocity generator to remove.
	 * @return true if the velocity generator has successfully been removed from the controller, false otherwise.
	 */
	bool removeVelocityGenerator(const std::string& name);

	/**
	 * @brief Remove a joint velocity generator from the controller.
	 * @param name The name given to the joint velocity generator to remove.
	 * @return true if the joint velocity generator has successfully been removed from the controller, false otherwise.
	 */
	bool removeJointVelocityGenerator(const std::string& name);

	/**
	 * @brief Retrieve a constraint from the controller.
	 * @param name The name given to the constraint to retreive.
	 * @return A pointer to the constraint. Store a null pointer if the constraint doesn't exist.
	 */
	std::shared_ptr<Constraint> getConstraint(const std::string& name);

	/**
	 * @brief Retrieve a force generator from the controller.
	 * @param name The name given to the force generator to retreive.
	 * @return A pointer to the force generator. Store a null pointer if the force generator doesn't exist.
	 */
	std::shared_ptr<ForceGenerator> getForceGenerator(const std::string& name);

	/**
	 * @brief Retrieve a torque generator from the controller.
	 * @param name The name given to the torque generator to retreive.
	 * @return A pointer to the torque generator. Store a null pointer if the torque generator doesn't exist.
	 */
	std::shared_ptr<TorqueGenerator> getTorqueGenerator(const std::string& name);

	/**
	 * @brief Retrieve a velocity generator from the controller.
	 * @param name The name given to the velocity generator to retreive.
	 * @return A pointer to the velocity generator. Store a null pointer if the velocity generator doesn't exist.
	 */
	std::shared_ptr<VelocityGenerator> getVelocityGenerator(const std::string& name);

	/**
	 * @brief Retrieve a joint velocity generator from the controller.
	 * @param name The name given to the joint velocity generator to retreive.
	 * @return A pointer to the joint velocity generator. Store a null pointer if the joint velocity generator doesn't exist.
	 */
	std::shared_ptr<JointVelocityGenerator> getJointVelocityGenerator(const std::string& name);

	/**
	 * @brief Use all the generators and constraints to compute the robot velocities
	 */
	void compute();

	/**
	 * @brief Call operator. Shortcut for compute().
	 */
	void operator()();

	/**
	 * @brief Print all the constraints and generators currently in use.
	 */
	void print() const;

	template<typename T>
	struct StorageWrapper {
		std::shared_ptr<T> object;
		typename std::result_of<decltype(&T::compute)(T)>::type last_value;
	};

	template<typename T>
	using storage_const_iterator = typename std::map<std::string,StorageWrapper<T>>::const_iterator;

	storage_const_iterator<Constraint> constraints_begin() const;
	storage_const_iterator<Constraint> constraints_end() const;

	storage_const_iterator<ForceGenerator> force_generators_begin() const;
	storage_const_iterator<ForceGenerator> force_generators_end() const;

	storage_const_iterator<TorqueGenerator> torque_generators_begin() const;
	storage_const_iterator<TorqueGenerator> torque_generators_end() const;

	storage_const_iterator<VelocityGenerator> velocity_generators_begin() const;
	storage_const_iterator<VelocityGenerator> velocity_generators_end() const;

	storage_const_iterator<JointVelocityGenerator> joint_velocity_generators_begin() const;
	storage_const_iterator<JointVelocityGenerator> joint_velocity_generators_end() const;


protected:
	/**
	 * @brief Construct a default safety controller. The robot must be set by the parent class.
	 */
	SafetyController();

	double computeConstraintValue();
	const Vector6d& computeForceSum();
	const VectorXd& computeTorqueSum();
	const Vector6d& computeVelocitySum();
	const VectorXd& computeJointVelocitySum();
	const MatrixXd& computeJacobianInverse() const;

	ObjectCollection<StorageWrapper<Constraint>>               constraints_;
	ObjectCollection<StorageWrapper<ForceGenerator>>           force_generators_;
	ObjectCollection<StorageWrapper<TorqueGenerator>>          torque_generators_;
	ObjectCollection<StorageWrapper<VelocityGenerator>>        velocity_generators_;
	ObjectCollection<StorageWrapper<JointVelocityGenerator>>   joint_velocity_generators_;

	RobotPtr robot_;
};

} // namespace RSCL
