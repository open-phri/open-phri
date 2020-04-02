#include <OpenPHRI/drivers/dummy_driver.h>
#include <OpenPHRI/utilities/exceptions.h>

#include <yaml-cpp/yaml.h>
#include <thread>
#include <chrono>
#include <iostream>

namespace phri {

bool DummyDriver::registered_in_factory =
    phri::DriverFactory::add<DummyDriver>("dummy");

DummyDriver::DummyDriver(phri::Robot& robot, double sample_time)
    : Driver(robot, sample_time),
      last_sync_{std::chrono::high_resolution_clock::now()} {
}

DummyDriver::DummyDriver(phri::Robot& robot, const YAML::Node& configuration)
    : Driver(robot, 0.) {
    const auto& driver = configuration["driver"];

    if (driver) {
        const auto& sample_time_node = driver["sample_time"];
        if (sample_time_node) {
            setTimeStep(sample_time_node.as<double>());
        } else {
            throw std::runtime_error(
                OPEN_PHRI_ERROR("You must provide a 'sample_time' field in the "
                                "driver configuration."));
        }
        auto init_joint_positions_node = driver["init_joint_positions"];
        if (init_joint_positions_node) {
            auto init_joint_positions =
                init_joint_positions_node.as<std::vector<double>>();
            if (init_joint_positions.size() != robot_.jointCount()) {
                throw std::runtime_error(OPEN_PHRI_ERROR(
                    "The number of values in 'init_joint_positions' does not "
                    "match the number of joints."));
            }
            for (size_t i = 0; i < robot_.jointCount(); ++i) {
                jointState().position()(i) =
                    init_joint_positions[i] * M_PI / 180.;
            }
        } else {
            jointState().position().setConstant(1.);
            std::cout << "\n"
                      << OPEN_PHRI_WARNING(
                             "'init_joint_positions' was not given, using ")
                      << jointState().position().transpose() << std::endl;
        }
    } else {
        throw std::runtime_error(OPEN_PHRI_ERROR(
            "The configuration file doesn't include a 'driver' field."));
    }
}

DummyDriver::~DummyDriver() = default;

bool DummyDriver::start(double timeout) {
    return true;
}

bool DummyDriver::stop() {
    return true;
}

bool DummyDriver::sync() {
    std::this_thread::sleep_until(
        last_sync_ +
        std::chrono::microseconds(static_cast<int>(getTimeStep() * 1e6)));
    last_sync_ = std::chrono::high_resolution_clock::now();
    return true;
}

bool DummyDriver::read() {
    return true;
}

bool DummyDriver::send() {
    jointState().position() += jointCommand().velocity() * getTimeStep();
    return true;
}

} // namespace phri
