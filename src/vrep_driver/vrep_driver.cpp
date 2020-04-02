/*      File: vrep_driver.cpp
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

#include <OpenPHRI/drivers/vrep_driver.h>
#include <OpenPHRI/utilities/exceptions.h>

#include <pid/rpath.h>
#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <sstream>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

#include <extApi.h>

namespace phri {

const bool VREPDriver::registered_in_factory =
    phri::DriverFactory::add<VREPDriver>("vrep");
std::map<std::string, int> VREPDriver::connection_to_client_id;

VREPDriver::VREPDriver(phri::Robot& robot, double sample_time,
                       const std::string& suffix, const std::string& ip,
                       int port)
    : phri::Driver(robot, sample_time), sync_mode_(true), suffix_(suffix) {
    start_parameters_.ip = ip;
    start_parameters_.port = port;
}

VREPDriver::VREPDriver(phri::Robot& robot, double sample_time, int client_id,
                       const std::string& suffix)
    : phri::Driver(robot, sample_time), sync_mode_(true), suffix_(suffix) {
    start_parameters_.client_id = client_id_;
}

VREPDriver::VREPDriver(phri::Robot& robot, const YAML::Node& configuration)
    : phri::Driver(robot, 0.) {
    const auto& vrep = configuration["driver"];

    if (vrep) {
        try {
            setTimeStep(vrep["sample_time"].as<double>());
        } catch (...) {
            throw std::runtime_error(
                OPEN_PHRI_ERROR("You must provide a 'sample_time' field in the "
                                "V-REP configuration."));
        }
        suffix_ = vrep["suffix"].as<std::string>("");
        sync_mode_ = vrep["synchronous"].as<bool>(true);
        auto mode = vrep["mode"].as<std::string>("TCP");
        if (mode == "TCP") {
            start_parameters_.ip = vrep["ip"].as<std::string>("127.0.0.1");
            start_parameters_.port = vrep["port"].as<int>(19997);
        } else if (mode == "shared_memory") {
            start_parameters_.port = vrep["port"].as<int>(-1000);
        } else {
            throw std::runtime_error(OPEN_PHRI_ERROR(
                "Invalid 'mode' field in the V-REP configuration. Value can be "
                "'TCP' or 'shared_memory'."));
        }
        auto scene = vrep["scene"].as<std::string>("");
        if (not scene.empty()) {
            setScene(scene);
        }
    } else {
        throw std::runtime_error(OPEN_PHRI_ERROR(
            "The configuration file doesn't include a 'driver' field."));
    }
}

VREPDriver::~VREPDriver() {
    enableSynchonous(false);
}

bool VREPDriver::init(double timeout) {
    spatial::Frame::save("world");

    bool ok = simxStartSimulation(client_id_, simx_opmode_oneshot_wait) ==
              simx_return_ok;

    enableSynchonous(sync_mode_);
    if (sync_mode_) {
        ok &= nextStep();
        ok &= nextStep();
    }

    ok &= Driver::init(timeout);

    return ok;
}

// void VREPDriver::init(const std::string& ip, int port) {
//     auto connection_name = ip + ":" + std::to_string(port);
//     try {
//         client_id_ = VREPDriver::connection_to_client_id.at(connection_name);
//     } catch (...) {
//         client_id_ = simxStart((simxChar*)ip.c_str(), port, 0, 1, 10000,
//                                int(getTimeStep() * 1000));
//         VREPDriver::connection_to_client_id[connection_name] = client_id_;
//     }

//     if (client_id_ != -1) {
//         return init(client_id_);
//     } else {
//         simxFinish(client_id_);
//         throw std::runtime_error(
//             "VREPDriver::init: can't initialize the connection with V-REP");
//     }
// }

// void VREPDriver::init(int client_id) {
//     assert_msg("In VREPDriver::init: invalid client id", client_id >= 0);
//     client_id_ = client_id;

//     getObjectHandles();
//     startStreaming();
// }

int VREPDriver::getClientID() const {
    return client_id_;
}

bool VREPDriver::checkConnection() const {
    return (simxGetConnectionId(client_id_) != -1);
}

bool VREPDriver::enableSynchonous(bool state) {
    sync_mode_ = state;
    return (simxSynchronous(client_id_, sync_mode_) == simx_return_ok);
}

bool VREPDriver::nextStep() const {
    if (sync_mode_) {
        return (simxSynchronousTrigger(client_id_) == simx_return_ok);
    }
    return false;
}

bool VREPDriver::sync() {
    return nextStep();
}

bool VREPDriver::start(double timeout) {
    if (start_parameters_.client_id >= 0) {
        client_id_ = start_parameters_.client_id;
    } else {
        auto connection_name =
            start_parameters_.ip + ":" + std::to_string(start_parameters_.port);
        if (auto connection_it =
                VREPDriver::connection_to_client_id.find(connection_name);
            connection_it != std::end(VREPDriver::connection_to_client_id)) {
            client_id_ = connection_it->second;
        } else {
            client_id_ = simxStart((simxChar*)start_parameters_.ip.c_str(),
                                   start_parameters_.port, 0, 1, 10000,
                                   int(getTimeStep() * 1000));
            VREPDriver::connection_to_client_id[connection_name] = client_id_;
        }
    }

    if (client_id_ >= 0) {
        if (not start_parameters_.scene_to_load.empty()) {
            loadScene(start_parameters_.scene_to_load);
        }

        getObjectHandles();
        startStreaming();
    } else {
        simxFinish(client_id_);
        throw std::runtime_error(
            "VREPDriver::init: failed to create a connection with V-REP");
    }

    return true;
} // namespace phri

bool VREPDriver::stop() {
    if (sync_mode_) {
        enableSynchonous(false);
    }
    return simxStopSimulation(client_id_, simx_opmode_oneshot_wait) ==
           simx_return_ok;
}

void VREPDriver::pause() {
    simxPauseSimulation(client_id_, simx_opmode_oneshot_wait);
}

bool VREPDriver::readTCPPose(spatial::Position& pose) const {
    bool all_ok = true;
    float data[7];

    int object_handle = object_handles_.at(robot().name() + "_tcp" + suffix_);
    int frame_id = getFrameHandle(pose.frame());
    all_ok &= (simxGetObjectPosition(client_id_, object_handle, frame_id, data,
                                     simx_opmode_buffer) == simx_return_ok);
    all_ok &=
        (simxGetObjectQuaternion(client_id_, object_handle, frame_id, data + 3,
                                 simx_opmode_buffer) == simx_return_ok);

    if (all_ok) {
        Eigen::Vector3d translation;
        Eigen::Quaterniond orientation;
        std::copy_n(data, 3, translation.data());
        std::copy_n(data + 3, 4, orientation.coeffs().data());
        pose.linear() = translation;
        pose.orientation() = orientation;
    }

    return all_ok;
}

bool VREPDriver::readTCPVelocity(spatial::Velocity& velocity) const {
    using namespace Eigen;

    bool all_ok = true;
    float data[6], angles[3];

    int object_handle = object_handles_.at(robot().name() + "_tcp" + suffix_);
    int frame_id = getFrameHandle(velocity.frame());
    all_ok &= (simxGetObjectOrientation(client_id_, frame_id, -1, angles,
                                        simx_opmode_buffer) == simx_return_ok);
    all_ok &= (simxGetObjectVelocity(client_id_, object_handle, data, data + 3,
                                     simx_opmode_buffer) == simx_return_ok);

    if (all_ok) {
        for (size_t i = 0; all_ok and i < 6; ++i) {
            velocity.data()[i] = data[i];
        }

        // With V-REP, the velocity is (sadly) always expressed in the absolute
        // frame so we need to map it from the absolute frame to the base frame
        Matrix3d rot_mat;
        rot_mat = AngleAxisd(angles[0], Vector3d::UnitX()) *
                  AngleAxisd(angles[1], Vector3d::UnitY()) *
                  AngleAxisd(angles[2], Vector3d::UnitZ());

        velocity.linear() = rot_mat.transpose() * velocity.linear();
        velocity.angular() = rot_mat.transpose() * velocity.angular();
    }

    return all_ok;
}

bool VREPDriver::readTCPWrench(spatial::Force& wrench) const {
    bool all_ok = true;
    float data[6];
    uint8_t ft_state;
    std::string obj_name = robot().name() + "_force_sensor" + suffix_;

    all_ok &= (simxReadForceSensor(client_id_, object_handles_.at(obj_name),
                                   &ft_state, data, data + 3,
                                   simx_opmode_buffer) == simx_return_ok);
    all_ok &= ft_state == 0b01; // ft not broken + data available

    if (all_ok) {
        for (size_t i = 0; all_ok and i < 6; ++i) {
            wrench.data()[i] = data[i];
        }
    }

    return all_ok;
}

std::shared_ptr<const spatial::Position>
VREPDriver::trackObjectPosition(const std::string& name,
                                const spatial::Frame& frame) {
    int handle = -1;
    int ref_frame = getFrameHandle(frame);
    float data[3];

    if (simxGetObjectHandle(client_id_, name.c_str(), &handle,
                            simx_opmode_oneshot_wait) != simx_return_ok) {
        throw std::runtime_error(
            OPEN_PHRI_ERROR("Can't get the handle of object " + name));
    }

    simxGetObjectPosition(client_id_, handle, ref_frame, data,
                          simx_opmode_streaming);

    auto ptr = std::make_shared<spatial::Position>(frame);

    tracked_objects_[std::make_pair(handle, ref_frame)] = ptr;

    return ptr;
}

bool VREPDriver::updateTrackedObjectsPosition() {
    bool all_ok = true;
    for (auto& obj : tracked_objects_) {
        float data[3];
        all_ok &= (simxGetObjectPosition(client_id_, obj.first.first,
                                         obj.first.second, data,
                                         simx_opmode_buffer) == simx_return_ok);
        if (all_ok) {
            Eigen::Vector3d translation;
            std::copy_n(data, 3, obj.second->linear().data());
        } else {
            throw std::runtime_error(
                OPEN_PHRI_ERROR("Can't get position of object with handle " +
                                std::to_string(obj.first.first)));
        }
    }
    return all_ok;
}

std::shared_ptr<const vector::dyn::Position>
VREPDriver::initLaserScanner(const std::string& name) {
    std::string data_name = name + "_data";
    simxUChar* sigVal;
    simxInt sigLen;

    simxReadStringStream(client_id_, data_name.c_str(), &sigVal, &sigLen,
                         simx_opmode_streaming);

    auto ptr = std::make_shared<vector::dyn::Position>();
    lasers_data_[data_name] = ptr;

    return ptr;
}

bool VREPDriver::updateLaserScanners() {
    bool all_ok = true;

    for (auto& laser : lasers_data_) {
        simxUChar* sigVal;
        simxInt sigLen;

        if (simxReadStringStream(client_id_, laser.first.c_str(), &sigVal,
                                 &sigLen,
                                 simx_opmode_buffer) == simx_return_ok) {
            size_t count = sigLen / sizeof(float);

            auto& vec = *laser.second;
            if (count != vec.size()) {
                vec.resize(count);
            }

            float* distances = reinterpret_cast<float*>(sigVal);
            for (size_t i = 0; i < count; ++i) {
                vec(i) = distances[i];
            }
        } else {
            all_ok = false;
        }
    }

    return all_ok;
}

bool VREPDriver::readJointPosition(vector::dyn::Position& position) {
    bool all_ok = true;

    float positions[robot().jointCount()];

    for (size_t i = 0; i < robot().jointCount(); ++i) {
        int joint_handle = object_handles_.at(robot().name() + "_joint" +
                                              std::to_string(i + 1) + suffix_);
        all_ok &= (simxGetJointPosition(client_id_, joint_handle, positions + i,
                                        simx_opmode_buffer) != -1);
    }
    double* position_data = position.data();
    for (size_t i = 0; all_ok and i < robot().jointCount(); ++i) {
        position_data[i] = positions[i];
    }

    return all_ok;
}

bool VREPDriver::sendJointTargetPosition(
    const vector::dyn::Position& position) {
    bool all_ok = true;

    for (size_t i = 0; i < robot().jointCount(); ++i) {
        int joint_handle = object_handles_.at(robot().name() + "_joint" +
                                              std::to_string(i + 1) + suffix_);
        all_ok &=
            (simxSetJointTargetPosition(client_id_, joint_handle, position(i),
                                        simx_opmode_oneshot) != -1);
    }

    return all_ok;
}

bool VREPDriver::sendJointTargetVelocity(
    const vector::dyn::Velocity& velocity) {
    bool all_ok = true;

    jointCommand().position() += velocity * scalar::Duration{getTimeStep()};

    all_ok &= sendJointTargetPosition(robot().joints().command().position());

    return all_ok;
}

bool VREPDriver::getObjectHandles() {
    bool all_ok = true;

    auto getHandle = [this](const std::string& name) -> bool {
        std::string obj_name = robot().name() + "_" + name + suffix_;
        bool ok = simxGetObjectHandle(
                      client_id_, obj_name.c_str(), &object_handles_[obj_name],
                      simx_opmode_oneshot_wait) == simx_return_ok;
        if (not ok) {
            throw std::runtime_error(
                OPEN_PHRI_ERROR("Can't get the handle of object " + obj_name));
        }
        return ok;
    };

    all_ok &= getHandle("tcp");
    all_ok &= getHandle("base_frame");
    all_ok &= getHandle("world_frame");
    all_ok &= getHandle("force_sensor");

    for (size_t i = 1; i <= robot().jointCount(); ++i) {
        all_ok &= getHandle("joint" + std::to_string(i));
    }

    return all_ok;
}

void VREPDriver::startStreaming() const {
    float data[6];

    spatial::Frame frames[] = {robot().controlPointFrame(),
                               robot().controlPointParentFrame(), worldFrame()};
    std::string objects[] = {"_tcp"};

    for (auto& object : objects) {
        int obj_handle = object_handles_.at(robot().name() + object + suffix_);
        for (auto frame : frames) {
            int frame_id = getFrameHandle(frame);
            simxGetObjectPosition(client_id_, obj_handle, frame_id, data,
                                  simx_opmode_streaming);
            simxGetObjectOrientation(client_id_, obj_handle, frame_id, data + 3,
                                     simx_opmode_streaming);
        }
        simxGetObjectVelocity(client_id_, obj_handle, data, data + 3,
                              simx_opmode_streaming);
    }

    for (auto frame : frames) {
        int frame_id = getFrameHandle(frame);
        simxGetObjectOrientation(client_id_, frame_id, -1, data,
                                 simx_opmode_streaming);
    }

    for (size_t i = 1; i <= robot().jointCount(); ++i) {
        int joint_handle = object_handles_.at(robot().name() + "_joint" +
                                              std::to_string(i) + suffix_);
        simxGetJointPosition(client_id_, joint_handle, data,
                             simx_opmode_streaming);
    }

    uint8_t ft_state;
    int obj_handle =
        object_handles_.at(robot().name() + "_force_sensor" + suffix_);
    simxReadForceSensor(client_id_, obj_handle, &ft_state, data, data + 3,
                        simx_opmode_streaming);

    simxUChar* jacobian_str;
    simxInt sLength;
    simxReadStringStream(client_id_, ("Jacobian-" + robot().name()).c_str(),
                         &jacobian_str, &sLength, simx_opmode_streaming);
    simxReadStringStream(client_id_, ("RotMat-" + robot().name()).c_str(),
                         &jacobian_str, &sLength, simx_opmode_streaming);
}

int VREPDriver::getFrameHandle(const spatial::Frame& frame) const {
    if (frame == robot().controlPointFrame()) {
        return object_handles_.at(robot().name() + "_tcp" + suffix_);
    } else if (frame == robot().controlPointParentFrame()) {
        return object_handles_.at(robot().name() + "_base_frame" + suffix_);
    } else if (frame == worldFrame()) {
        return object_handles_.at(robot().name() + "_world_frame" + suffix_);
    }
    return -1;
}

bool VREPDriver::read() {
    bool all_ok = true;
    if (sync_mode_) {
        all_ok &= nextStep();
    } else {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(int(getTimeStep() * 1000.)));
    }

    all_ok &= readTCPVelocity(taskState().velocity());
    all_ok &= readTCPWrench(taskState().force());
    all_ok &= readJointPosition(jointState().position());
    all_ok &= updateTrackedObjectsPosition();
    all_ok &= updateLaserScanners();

    return all_ok;
}

bool VREPDriver::send() {
    bool all_ok = true;

    // Make sure all commands are sent at the same time
    simxPauseCommunication(client_id_, true);

    all_ok &= sendJointTargetVelocity(robot().joints().command().velocity());

    simxPauseCommunication(client_id_, false);

    return all_ok;
}

bool VREPDriver::isRegisteredInFactory() {
    return registered_in_factory;
}

void VREPDriver::setScene(const std::string& path) {
    start_parameters_.scene_to_load = PID_PATH(path);
}

void VREPDriver::loadScene(const std::string& path) const {
    simxLoadScene(client_id_, path.c_str(), 1, simx_opmode_blocking);
}

} // namespace phri
