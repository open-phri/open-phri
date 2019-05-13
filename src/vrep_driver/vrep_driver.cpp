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

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <sstream>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

#include <extApi.h>
#include <v_repConst.h>

using namespace phri;
using namespace std;

const bool VREPDriver::registered_in_factory =
    phri::DriverFactory::add<VREPDriver>("vrep");
std::map<std::string, int> VREPDriver::connection_to_client_id;

VREPDriver::VREPDriver(phri::Robot& robot, double sample_time,
                       const std::string& suffix, const std::string& ip,
                       int port)
    : phri::Driver(robot, sample_time), sync_mode_(true), suffix_(suffix) {
    init(ip, port);
}

VREPDriver::VREPDriver(phri::Robot& robot, double sample_time, int client_id,
                       const std::string& suffix)
    : phri::Driver(robot, sample_time), sync_mode_(true), suffix_(suffix) {
    init(client_id);
}

VREPDriver::VREPDriver(phri::Robot& robot, const YAML::Node& configuration)
    : phri::Driver(robot, 0.) {
    const auto& vrep = configuration["driver"];

    if (vrep) {
        try {
            robot_.control.time_step = vrep["sample_time"].as<double>();
        } catch (...) {
            throw std::runtime_error(
                OPEN_PHRI_ERROR("You must provide a 'sample_time' field in the "
                                "V-REP configuration."));
        }
        suffix_ = vrep["suffix"].as<std::string>("");
        sync_mode_ = vrep["synchronous"].as<bool>(true);
        auto mode = vrep["mode"].as<std::string>("TCP");
        if (mode == "TCP") {
            std::string ip = vrep["ip"].as<std::string>("127.0.0.1");
            int port = vrep["port"].as<int>(19997);
            init(ip, port);
        } else if (mode == "shared_memory") {
            int port = vrep["port"].as<int>(-1000);
            init("", port);
        } else {
            throw std::runtime_error(OPEN_PHRI_ERROR(
                "Invalid 'mode' field in the V-REP configuration. Value can be "
                "'TCP' or 'shared_memory'."));
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
    enableSynchonous(sync_mode_);
    return Driver::init(timeout);
}

void VREPDriver::init(const std::string& ip, int port) {
    auto connection_name = ip + ":" + std::to_string(port);
    try {
        client_id_ = VREPDriver::connection_to_client_id.at(connection_name);
    } catch (...) {
        client_id_ = simxStart((simxChar*)ip.c_str(), port, 0, 1, 10000,
                               int(getSampleTime() * 1000));
        VREPDriver::connection_to_client_id[connection_name] = client_id_;
    }

    if (client_id_ != -1) {
        return init(client_id_);
    } else {
        simxFinish(client_id_);
        throw std::runtime_error(
            "VREPDriver::init: can't initialize the connection with V-REP");
    }
}

void VREPDriver::init(int client_id) {
    assert_msg("In VREPDriver::init: invalid client id", client_id >= 0);
    client_id_ = client_id;

    getObjectHandles();
    startStreaming();
}

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
    bool ok = simxStartSimulation(client_id_, simx_opmode_oneshot_wait) ==
              simx_return_ok;
    if (sync_mode_) {
        ok &= nextStep();
        ok &= nextStep();
    }
    return ok;
}

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

bool VREPDriver::readTCPPose(phri::Pose& pose,
                             phri::ReferenceFrame frame) const {
    bool all_ok = true;
    float data[6];

    int object_handle = object_handles_.at(robot_.name() + "_tcp" + suffix_);
    int frame_id = getFrameHandle(frame);
    all_ok &= (simxGetObjectPosition(client_id_, object_handle, frame_id, data,
                                     simx_opmode_buffer) == simx_return_ok);
    all_ok &=
        (simxGetObjectOrientation(client_id_, object_handle, frame_id, data + 3,
                                  simx_opmode_buffer) == simx_return_ok);

    if (all_ok) {
        phri::Vector6d pose_vec;
        for (size_t i = 0; all_ok and i < 6; ++i) {
            pose_vec[i] = data[i];
        }
        pose = pose_vec;
    }

    return all_ok;
}

bool VREPDriver::readTCPVelocity(phri::Twist& velocity,
                                 phri::ReferenceFrame frame) const {
    using namespace Eigen;

    bool all_ok = true;
    float data[6], angles[3];

    int object_handle = object_handles_.at(robot_.name() + "_tcp" + suffix_);
    int frame_id = getFrameHandle(frame);
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

        velocity.translation() = rot_mat.transpose() * velocity.translation();
        velocity.rotation() = rot_mat.transpose() * velocity.rotation();
    }

    return all_ok;
}

bool VREPDriver::readTCPWrench(phri::Wrench& wrench) const {
    bool all_ok = true;
    float data[6];
    uint8_t ft_state;
    string obj_name = robot_.name() + "_force_sensor" + suffix_;

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

bool VREPDriver::readJacobian(phri::MatrixXd& jacobian) const {
    bool all_ok = false;

    simxUChar* jacobian_buf;
    simxInt sLength;
    int ret =
        simxReadStringStream(client_id_, ("Jacobian-" + robot_.name()).c_str(),
                             &jacobian_buf, &sLength, simx_opmode_buffer);
    if (ret == simx_return_ok) {
        if (sLength == 0) {
            return false;
        }

        std::string jacobian_str = std::string((char*)(jacobian_buf));
        std::istringstream iss(jacobian_str);

        size_t rows, cols;
        iss >> rows;
        iss >> cols;
        jacobian.resize(rows, cols);
        for (size_t idx = 0; idx < rows * cols; ++idx) {
            size_t r = idx / cols, c = idx % cols;
            iss >> jacobian(r, c);
        }
        // Jacobians in V-REP are transposed compared to the standard form and
        // with joints in the tip-to-base order so we fix all that
        jacobian = jacobian.transpose().rowwise().reverse().eval();
        all_ok = true;
    }
    // else {
    //  std::cerr << "JACOBIAN ERROR! ret: " << ret << "\n";
    // }

    return all_ok;
}

bool VREPDriver::readTransformationMatrix(phri::Matrix4d& matrix) const {
    bool all_ok = false;

    simxUChar* matrix_buf;
    simxInt sLength;
    if (simxReadStringStream(client_id_, ("RotMat-" + robot_.name()).c_str(),
                             &matrix_buf, &sLength,
                             simx_opmode_buffer) == simx_return_ok) {
        std::string matrix_str = std::string((char*)(matrix_buf));
        std::istringstream iss(matrix_str);

        matrix.setIdentity();
        for (size_t row = 0; row < 3; ++row) {
            for (size_t col = 0; col < 4; ++col) {
                iss >> matrix(row, col);
            }
        }
        all_ok = true;
    }

    return all_ok;
}

std::shared_ptr<const Pose>
VREPDriver::trackObjectPosition(const std::string& name,
                                phri::ReferenceFrame frame) {
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

    auto ptr = make_shared<phri::Pose>();

    tracked_objects_[make_pair(handle, ref_frame)] = ptr;

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
            phri::Vector6d pose_vec;
            for (size_t i = 0; all_ok and i < 3; ++i) {
                pose_vec[i] = data[i];
            }
            *obj.second = pose_vec;
        } else {
            throw std::runtime_error(
                OPEN_PHRI_ERROR("Can't get position of object with handle " +
                                std::to_string(obj.first.first)));
        }
    }
    return all_ok;
}

std::shared_ptr<const VectorXd>
VREPDriver::initLaserScanner(const std::string& name) {
    std::string data_name = name + "_data";
    simxUChar* sigVal;
    simxInt sigLen;

    simxReadStringStream(client_id_, data_name.c_str(), &sigVal, &sigLen,
                         simx_opmode_streaming);

    auto ptr = std::make_shared<phri::VectorXd>();
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

bool VREPDriver::readJointPosition(phri::VectorXd& position) const {
    bool all_ok = true;

    float positions[robot_.jointCount()];

    for (size_t i = 0; i < robot_.jointCount(); ++i) {
        int joint_handle = object_handles_.at(robot_.name() + "_joint" +
                                              std::to_string(i + 1) + suffix_);
        all_ok &= (simxGetJointPosition(client_id_, joint_handle, positions + i,
                                        simx_opmode_buffer) != -1);
    }
    double* position_data = position.data();
    for (size_t i = 0; all_ok and i < robot_.jointCount(); ++i) {
        position_data[i] = positions[i];
    }

    return all_ok;
}

bool VREPDriver::sendJointTargetPosition(const phri::VectorXd& position) const {
    bool all_ok = true;

    for (size_t i = 0; i < robot_.jointCount(); ++i) {
        int joint_handle = object_handles_.at(robot_.name() + "_joint" +
                                              std::to_string(i + 1) + suffix_);
        all_ok &=
            (simxSetJointTargetPosition(client_id_, joint_handle, position(i),
                                        simx_opmode_oneshot) != -1);
    }

    return all_ok;
}

bool VREPDriver::sendJointTargetVelocity(const phri::VectorXd& velocity) const {
    bool all_ok = true;

    robot_.joints.command.position += velocity * getSampleTime();

    all_ok &= sendJointTargetPosition(robot_.joints.command.position);

    return all_ok;
}

bool VREPDriver::getObjectHandles() {
    bool all_ok = true;

    auto getHandle = [this](const std::string& name) -> bool {
        string obj_name = robot_.name() + "_" + name + suffix_;
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

    for (size_t i = 1; i <= robot_.jointCount(); ++i) {
        all_ok &= getHandle("joint" + std::to_string(i));
    }

    return all_ok;
}

void VREPDriver::startStreaming() const {
    float data[6];

    phri::ReferenceFrame frames[] = {phri::ReferenceFrame::TCP,
                                     phri::ReferenceFrame::Base,
                                     phri::FrameAdapter::world()};
    string objects[] = {"_tcp"};

    for (auto& object : objects) {
        int obj_handle = object_handles_.at(robot_.name() + object + suffix_);
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

    for (size_t i = 1; i <= robot_.jointCount(); ++i) {
        int joint_handle = object_handles_.at(robot_.name() + "_joint" +
                                              std::to_string(i) + suffix_);
        simxGetJointPosition(client_id_, joint_handle, data,
                             simx_opmode_streaming);
    }

    uint8_t ft_state;
    int obj_handle =
        object_handles_.at(robot_.name() + "_force_sensor" + suffix_);
    simxReadForceSensor(client_id_, obj_handle, &ft_state, data, data + 3,
                        simx_opmode_streaming);

    simxUChar* jacobian_str;
    simxInt sLength;
    simxReadStringStream(client_id_, ("Jacobian-" + robot_.name()).c_str(),
                         &jacobian_str, &sLength, simx_opmode_streaming);
    simxReadStringStream(client_id_, ("RotMat-" + robot_.name()).c_str(),
                         &jacobian_str, &sLength, simx_opmode_streaming);
}

int VREPDriver::getFrameHandle(phri::ReferenceFrame frame) const {
    switch (frame) {
    case phri::ReferenceFrame::TCP:
        return object_handles_.at(robot_.name() + "_tcp" + suffix_);
        break;
    case phri::ReferenceFrame::Base:
        return object_handles_.at(robot_.name() + "_base_frame" + suffix_);
        break;
    case phri::FrameAdapter::world():
        return object_handles_.at(robot_.name() + "_world_frame" + suffix_);
        break;
    }
    return -1;
}

bool VREPDriver::read() {
    bool all_ok = true;
    if (sync_mode_) {
        all_ok &= nextStep();
    } else {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(int(getSampleTime() * 1000.)));
    }

    all_ok &=
        readTCPVelocity(robot_.task.state.twist, phri::ReferenceFrame::Base);
    all_ok &= readTCPWrench(robot_.task.state.wrench);
    all_ok &= readJointPosition(robot_.joints.state.position);
    all_ok &= updateTrackedObjectsPosition();
    all_ok &= updateLaserScanners();

    return all_ok;
}

bool VREPDriver::send() {
    bool all_ok = true;

    // Make sure all commands are sent at the same time
    simxPauseCommunication(client_id_, true);

    all_ok &= sendJointTargetVelocity(robot_.joints.command.velocity);

    simxPauseCommunication(client_id_, false);

    return all_ok;
}

bool VREPDriver::isRegisteredInFactory() {
    return registered_in_factory;
}
