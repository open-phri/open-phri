/*      File: safety_controller.h
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
 * @file safety_controller.h
 * @author Benjamin Navarro
 * @brief Definition of the SafetyController class
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/robot.h>
#include <OpenPHRI/fwd_decl.h>
#include <OpenPHRI/utilities/object_collection.hpp>

#include <map>
#include <type_traits>

namespace phri {

/** @brief The controller class. Deals with all the generators (velocity &
 * force) and the constraints to compute the TCP velocity command.
 *  @details Provides mechanisms to add/get/remove constraints and generators
 * (can be done at any time) and to get the velocity command as well as
 * intermediate computation values (mainly used by constraints).
 */
class SafetyController {
public:
    /**
     * @brief Construct a safety controller with a given robot.
     * @param robot The robot to work with.
     */
    explicit SafetyController(Robot& robot);
    /**
     * @brief Construct a safety controller with a given robot and a specific
     * configuration.
     * @param robot The robot to work with.
     * @param configuration The YAML node containing the controller's
     * configuration.
     */
    explicit SafetyController(Robot& robot, YAML::Node& configuration);

    SafetyController(SafetyController&&) = default;
    ~SafetyController() = default;

    /**
     * @brief Set the verbosity on or off. If on, error messages can be printed
     * when calling the add/get/remove methods.
     * @param on True to enable verbosity, false to disable it.
     */
    void setVerbose(bool on);

    /**
     * @brief This can be used to disable the Jacobian inverse computation in
     * the case it is performed externally and updated on the robot object.
     * @param on True to skip the computation, false to perform it.
     */
    void skipJacobianInverseComputation(bool on);

    /**
     * @brief Add a new constraint to the controller.
     * @param name The name given to the constraint. Must be unique. Used to
     * latter get/remove the constraint.
     * @param constraint A shared pointer to the constraint to add.
     * @param force [optional] If true, any constraint with the same name will
     * be overriden.
     * @return true if the constraint has successfully been added to the
     * controller, false otherwise.
     */
    bool addConstraint(const std::string& name,
                       std::shared_ptr<Constraint> constraint,
                       bool force = false);

    /**
     * @brief Add a new force generator to the controller.
     * @param name The name given to the force generator. Must be unique. Used
     * to latter get/remove the force generator.
     * @param generator A shared pointer to the force generator to add.
     * @param force [optional] If true, any force generator with the same name
     * will be overriden.
     * @return true if the force generator has successfully been added to the
     * controller, false otherwise.
     */
    bool addForceGenerator(const std::string& name,
                           std::shared_ptr<ForceGenerator> generator,
                           bool force = false);

    /**
     * @brief Add a new torque generator to the controller.
     * @param name The name given to the torque generator. Must be unique. Used
     * to latter get/remove the torque generator.
     * @param generator A shared pointer to the torque generator to add.
     * @param force [optional] If true, any force generator with the same name
     * will be overriden.
     * @return true if the torque generator has successfully been added to the
     * controller, false otherwise.
     */
    bool addJointForceGenerator(const std::string& name,
                                std::shared_ptr<JointForceGenerator> generator,
                                bool force = false);

    /**
     * @brief Add a new velocity generator to the controller.
     * @param name The name given to the velocity generator. Must be unique.
     * Used to latter get/remove the velocity generator.
     * @param generator A shared pointer to the velocity generator to add.
     * @param force [optional] If true, any velocity generator with the same
     * name will be overriden.
     * @return true if the velocity generator has successfully been added to the
     * controller, false otherwise.
     */
    bool addVelocityGenerator(const std::string& name,
                              std::shared_ptr<VelocityGenerator> generator,
                              bool force = false);

    /**
     * @brief Add a new joint velocity generator to the controller.
     * @param name The name given to the joint velocity generator. Must be
     * unique. Used to latter get/remove the joint velocity generator.
     * @param generator A shared pointer to the joint velocity generator to add.
     * @param force [optional] If true, any joint velocity generator with the
     * same name will be overriden.
     * @return true if the joint velocity generator has successfully been added
     * to the controller, false otherwise.
     */
    bool
    addJointVelocityGenerator(const std::string& name,
                              std::shared_ptr<JointVelocityGenerator> generator,
                              bool force = false);

    /**
     * @brief Shortcut for the SafetyController::addConstraint method.
     */
    template <typename T>
    typename std::enable_if<std::is_base_of<Constraint, T>::value, bool>::type
    add(const std::string& name, const std::shared_ptr<T>& obj,
        bool force = false) {
        return addConstraint(name, obj, force);
    }

    /**
     * @brief Shortcut for the SafetyController::addConstraint method.
     */
    template <typename T>
    typename std::enable_if<std::is_base_of<Constraint, T>::value, bool>::type
    add(const std::string& name, T&& obj, bool force = false) {
        return addConstraint(name, std::make_shared<T>(std::move(obj)), force);
    }

    template <typename T, typename... Args>
    typename std::enable_if<std::is_base_of<Constraint, T>::value, bool>::type
    add(const std::string& name, Args&&... args) {
        return addConstraint(
            name, std::make_shared<T>(std::forward<Args>(args)...), false);
    }

    /**
     * @brief Shortcut for the SafetyController::addForceGenerator method.
     */
    template <typename T>
    typename std::enable_if<std::is_base_of<ForceGenerator, T>::value,
                            bool>::type
    add(const std::string& name, const std::shared_ptr<T>& obj,
        bool force = false) {
        return addForceGenerator(name, obj, force);
    }

    /**
     * @brief Shortcut for the SafetyController::addForceGenerator method.
     */
    template <typename T>
    typename std::enable_if<std::is_base_of<ForceGenerator, T>::value,
                            bool>::type
    add(const std::string& name, T&& obj, bool force = false) {
        return addForceGenerator(name, std::make_shared<T>(std::move(obj)),
                                 force);
    }

    template <typename T, typename... Args>
    typename std::enable_if<std::is_base_of<ForceGenerator, T>::value,
                            bool>::type
    add(const std::string& name, Args&&... args) {
        return addForceGenerator(
            name, std::make_shared<T>(std::forward<Args>(args)...), false);
    }

    /**
     * @brief Shortcut for the SafetyController::addJointForceGenerator method.
     */
    template <typename T>
    typename std::enable_if<std::is_base_of<JointForceGenerator, T>::value,
                            bool>::type
    add(const std::string& name, const std::shared_ptr<T>& obj,
        bool force = false) {
        return addJointForceGenerator(name, obj, force);
    }

    /**
     * @brief Shortcut for the SafetyController::addJointForceGenerator method.
     */
    template <typename T>
    typename std::enable_if<std::is_base_of<JointForceGenerator, T>::value,
                            bool>::type
    add(const std::string& name, T&& obj, bool force = false) {
        return addJointForceGenerator(name, std::make_shared<T>(std::move(obj)),
                                      force);
    }

    template <typename T, typename... Args>
    typename std::enable_if<std::is_base_of<JointForceGenerator, T>::value,
                            bool>::type
    add(const std::string& name, Args&&... args) {
        return addJointForceGenerator(
            name, std::make_shared<T>(std::forward<Args>(args)...), false);
    }

    /**
     * @brief Shortcut for the SafetyController::addVelocityGenerator method.
     */
    template <typename T>
    typename std::enable_if<std::is_base_of<VelocityGenerator, T>::value,
                            bool>::type
    add(const std::string& name, const std::shared_ptr<T>& obj,
        bool force = false) {
        return addVelocityGenerator(name, obj, force);
    }

    /**
     * @brief Shortcut for the SafetyController::addVelocityGenerator method.
     */
    template <typename T>
    typename std::enable_if<std::is_base_of<VelocityGenerator, T>::value,
                            bool>::type
    add(const std::string& name, T&& obj, bool force = false) {
        return addVelocityGenerator(name, std::make_shared<T>(std::move(obj)),
                                    force);
    }

    template <typename T, typename... Args>
    typename std::enable_if<std::is_base_of<VelocityGenerator, T>::value,
                            bool>::type
    add(const std::string& name, Args&&... args) {
        return addVelocityGenerator(
            name, std::make_shared<T>(std::forward<Args>(args)...), false);
    }

    /**
     * @brief Shortcut for the SafetyController::addJointVelocityGenerator
     * method.
     */
    template <typename T>
    typename std::enable_if<std::is_base_of<JointVelocityGenerator, T>::value,
                            bool>::type
    add(const std::string& name, const std::shared_ptr<T>& obj,
        bool force = false) {
        return addJointVelocityGenerator(name, obj, force);
    }

    /**
     * @brief Shortcut for the SafetyController::addJointVelocityGenerator
     * method.
     */
    template <typename T>
    typename std::enable_if<std::is_base_of<JointVelocityGenerator, T>::value,
                            bool>::type
    add(const std::string& name, T&& obj, bool force = false) {
        return addJointVelocityGenerator(
            name, std::make_shared<T>(std::move(obj)), force);
    }

    template <typename T, typename... Args>
    typename std::enable_if<std::is_base_of<JointVelocityGenerator, T>::value,
                            bool>::type
    add(const std::string& name, Args&&... args) {
        return addJointVelocityGenerator(
            name, std::make_shared<T>(std::forward<Args>(args)...), false);
    }

    /**
     * @brief Remove a constraint from the controller.
     * @param name The name given to the constraint to remove.
     * @return true if the constraint has successfully been removed from the
     * controller, false otherwise.
     */
    bool removeConstraint(const std::string& name);

    /**
     * @brief Remove a force generator from the controller.
     * @param name The name given to the force generator to remove.
     * @return true if the force generator has successfully been removed from
     * the controller, false otherwise.
     */
    bool removeForceGenerator(const std::string& name);

    /**
     * @brief Remove a torque generator from the controller.
     * @param name The name given to the torque generator to remove.
     * @return true if the torque generator has successfully been removed from
     * the controller, false otherwise.
     */
    bool removeJointForceGenerator(const std::string& name);

    /**
     * @brief Remove a velocity generator from the controller.
     * @param name The name given to the velocity generator to remove.
     * @return true if the velocity generator has successfully been removed from
     * the controller, false otherwise.
     */
    bool removeVelocityGenerator(const std::string& name);

    /**
     * @brief Remove a joint velocity generator from the controller.
     * @param name The name given to the joint velocity generator to remove.
     * @return true if the joint velocity generator has successfully been
     * removed from the controller, false otherwise.
     */
    bool removeJointVelocityGenerator(const std::string& name);

    /**
     * @brief Retrieve a constraint from the controller.
     * @param name The name given to the constraint to retreive.
     * @return A pointer to the constraint. Store a null pointer if the
     * constraint doesn't exist.
     */
    std::shared_ptr<Constraint> getConstraint(const std::string& name);

    /**
     * @brief Retrieve a force generator from the controller.
     * @param name The name given to the force generator to retreive.
     * @return A pointer to the force generator. Store a null pointer if the
     * force generator doesn't exist.
     */
    std::shared_ptr<ForceGenerator> getForceGenerator(const std::string& name);

    /**
     * @brief Retrieve a torque generator from the controller.
     * @param name The name given to the torque generator to retreive.
     * @return A pointer to the torque generator. Store a null pointer if the
     * torque generator doesn't exist.
     */
    std::shared_ptr<JointForceGenerator>
    getJointForceGenerator(const std::string& name);

    /**
     * @brief Retrieve a velocity generator from the controller.
     * @param name The name given to the velocity generator to retreive.
     * @return A pointer to the velocity generator. Store a null pointer if the
     * velocity generator doesn't exist.
     */
    std::shared_ptr<VelocityGenerator>
    getVelocityGenerator(const std::string& name);

    /**
     * @brief Retrieve a joint velocity generator from the controller.
     * @param name The name given to the joint velocity generator to retreive.
     * @return A pointer to the joint velocity generator. Store a null pointer
     * if the joint velocity generator doesn't exist.
     */
    std::shared_ptr<JointVelocityGenerator>
    getJointVelocityGenerator(const std::string& name);

    /**
     * @brief Shortcut for the SafetyController::getConstraint method, with
     * pointer type conversion.
     */
    template <typename T>
    std::shared_ptr<T>
    get(const std::string& name,
        typename std::enable_if<std::is_base_of<Constraint, T>::value>::type* =
            nullptr) {
        return std::dynamic_pointer_cast<T>(getConstraint(name));
    }

    /**
     * @brief Shortcut for the SafetyController::getForceGenerator method, with
     * pointer type conversion.
     */
    template <typename T>
    std::shared_ptr<T>
    get(const std::string& name,
        typename std::enable_if<
            std::is_base_of<ForceGenerator, T>::value>::type* = nullptr) {
        return std::dynamic_pointer_cast<T>(getForceGenerator(name));
    }

    /**
     * @brief Shortcut for the SafetyController::getJointForceGenerator method,
     * with pointer type conversion.
     */
    template <typename T>
    std::shared_ptr<T>
    get(const std::string& name,
        typename std::enable_if<
            std::is_base_of<JointForceGenerator, T>::value>::type* = nullptr) {
        return getJointForceGenerator(name);
    }

    /**
     * @brief Shortcut for the SafetyController::getVelocityGenerator method,
     * with pointer type conversion.
     */
    template <typename T>
    std::shared_ptr<T>
    get(const std::string& name,
        typename std::enable_if<
            std::is_base_of<VelocityGenerator, T>::value>::type* = nullptr) {
        return getVelocityGenerator(name);
    }

    /**
     * @brief Shortcut for the SafetyController::getJointVelocityGenerator
     * method, with pointer type conversion.
     */
    template <typename T>
    std::shared_ptr<T>
    get(const std::string& name,
        typename std::enable_if<std::is_base_of<JointVelocityGenerator,
                                                T>::value>::type* = nullptr) {
        return getJointVelocityGenerator(name);
    }

    void removeAll();
    void removeAllVelocityInputs();
    void removeAllJointVelocityInputs();
    void removeAllForceInputs();
    void removeAllTorqueInputs();
    void removeAllConstraints();

    void enableDampedLeastSquares(double lambda);

    void enableDynamicDampedLeastSquares(double lambda_max,
                                         double sigma_min_threshold);

    /**
     * @brief Use all the generators and constraints to compute the robot
     * velocities
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

    template <typename T> struct StorageWrapper {
        template <typename U>
        using remove_cref_t = std::remove_const_t<std::remove_reference_t<U>>;

        using value_type = remove_cref_t<decltype(std::declval<T>().compute())>;

        template <typename U = T>
        StorageWrapper(typename std::enable_if<
                           std::is_same<VelocityGenerator, U>::value or
                           std::is_same<ForceGenerator, U>::value>::type* = 0)
            : last_value{spatial::Frame(uint64_t{0})} {
        }

        template <typename U = T>
        StorageWrapper(
            typename std::enable_if<
                not std::is_same<VelocityGenerator, U>::value and
                not std::is_same<ForceGenerator, U>::value>::type* = 0)
            : last_value{} {
        }

        StorageWrapper(std::shared_ptr<T> obj,
                       const value_type& value = value_type{})
            : object{obj}, last_value{value} {
        }

        StorageWrapper(const StorageWrapper&) = default;
        StorageWrapper(StorageWrapper&&) = default;
        ~StorageWrapper() = default;

        StorageWrapper& operator=(const StorageWrapper&) = default;
        StorageWrapper& operator=(StorageWrapper&&) = default;

        std::shared_ptr<T> object;
        value_type last_value;
    };

    template <typename T>
    using storage_const_iterator =
        typename std::map<std::string, StorageWrapper<T>>::const_iterator;

    storage_const_iterator<Constraint> constraints_begin() const;
    storage_const_iterator<Constraint> constraints_end() const;

    storage_const_iterator<ForceGenerator> force_generators_begin() const;
    storage_const_iterator<ForceGenerator> force_generators_end() const;

    storage_const_iterator<JointForceGenerator> torque_generators_begin() const;
    storage_const_iterator<JointForceGenerator> torque_generators_end() const;

    storage_const_iterator<VelocityGenerator> velocity_generators_begin() const;
    storage_const_iterator<VelocityGenerator> velocity_generators_end() const;

    storage_const_iterator<JointVelocityGenerator>
    joint_velocity_generators_begin() const;
    storage_const_iterator<JointVelocityGenerator>
    joint_velocity_generators_end() const;

protected:
    double computeConstraintValue();
    const spatial::Force& computeForceSum();
    const vector::dyn::Force& computeTorqueSum();
    const spatial::Velocity& computeVelocitySum();
    const vector::dyn::Velocity& computeJointVelocitySum();
    const Eigen::MatrixXd& computeJacobianInverse() const;

    ObjectCollection<StorageWrapper<Constraint>> constraints_;
    ObjectCollection<StorageWrapper<ForceGenerator>> force_generators_;
    ObjectCollection<StorageWrapper<JointForceGenerator>> torque_generators_;
    ObjectCollection<StorageWrapper<VelocityGenerator>> velocity_generators_;
    ObjectCollection<StorageWrapper<JointVelocityGenerator>>
        joint_velocity_generators_;

    Robot& robot_;
    bool skip_jacobian_inverse_computation_;

    bool dynamic_dls_;
    double lambda2_;
    double sigma_min_threshold_;
};

} // namespace phri
