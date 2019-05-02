/*      File: robot.cpp
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

#include <OpenPHRI/robot.h>
#include <OpenPHRI/utilities/exceptions.h>

#include <yaml-cpp/yaml.h>

namespace phri {

Robot::Robot(const std::string& name, size_t joint_count) {
    create(name, joint_count);
}

Robot::Robot(const YAML::Node& configuration) {
    create(configuration);
}

void Robot::create(const std::string& name, size_t joint_count) {
    name_ = name;
    joint_count_ = joint_count;
    joints.resize(joint_count_);
    control.resize(joint_count_);
}

void Robot::create(const YAML::Node& configuration) {
    const auto& robot = configuration["robot"];
    std::string name;
    size_t joint_count;
    if (robot) {
        try {
            name = robot["name"].as<std::string>();
        } catch (...) {
            throw std::runtime_error(OPEN_PHRI_ERROR(
                "You must provide a 'name' field in the robot configuration."));
        }
        try {
            joint_count = robot["joint_count"].as<size_t>();
        } catch (...) {
            throw std::runtime_error(
                OPEN_PHRI_ERROR("You must provide a 'joint_count' field in the "
                                "robot configuration."));
        }

        create(name, joint_count);
    } else {
        throw std::runtime_error(OPEN_PHRI_ERROR(
            "The configuration file doesn't include a 'robot' field."));
    }
}

const std::string& Robot::name() const {
    return name_;
}

size_t Robot::jointCount() const {
    return joint_count_;
}

} // namespace phri
