#include <OpenPHRI/drivers/dummy_driver.h>
#include <OpenPHRI/utilities/exceptions.h>

#include <yaml-cpp/yaml.h>
#include <thread>
#include <chrono>

using namespace phri;
using namespace std;

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
        try {
            setTimeStep(driver["sample_time"].as<double>());
        } catch (...) {
            throw std::runtime_error(
                OPEN_PHRI_ERROR("You must provide a 'sample_time' field in the "
                                "driver configuration."));
        }
        try {
            auto init_joint_positions =
                driver["init_joint_positions"].as<std::vector<double>>();
            if (init_joint_positions.size() != robot_.jointCount()) {
                throw std::runtime_error(OPEN_PHRI_ERROR(
                    "The number of values in 'init_joint_positions' does not "
                    "match the number of joints."));
            }
            for (size_t i = 0; i < robot_.jointCount(); ++i) {
                jointState().position()(i) =
                    init_joint_positions[i] * M_PI / 180.;
            }
        } catch (...) {
            throw std::runtime_error(
                OPEN_PHRI_ERROR("You must provide a 'init_joint_positions' "
                                "field in the driver configuration."));
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
