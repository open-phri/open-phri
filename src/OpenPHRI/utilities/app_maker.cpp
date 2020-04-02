#include <OpenPHRI/utilities/app_maker.h>
#include <OpenPHRI/utilities/clock.h>
#include <OpenPHRI/constraints/constraint.h>
#include <OpenPHRI/velocity_generators/velocity_generator.h>
#include <OpenPHRI/force_generators/force_generator.h>
#include <OpenPHRI/joint_velocity_generators/joint_velocity_generator.h>
#include <OpenPHRI/joint_force_generators/joint_force_generator.h>

#include <pid/rpath.h>

#include <optional>

namespace phri {

struct AppMaker::pImpl {
    std::optional<Robot> robot;
    std::optional<SafetyController> controller;
    std::optional<RobotModel> model;
    std::shared_ptr<Driver> driver;
    std::optional<DataLogger> data_logger;
    std::optional<Clock> clock;
    YAML::Node app_configuration;
    double init_timeout;
    double start_timeout;
    bool initialized;
};

AppMaker::AppMaker(const std::string& configuration_file)
    : impl_(std::make_unique<AppMaker::pImpl>()) {
    std::cout << "[phri::AppMaker] Parsing the configuration file..."
              << std::flush;
    auto conf = YAML::LoadFile(PID_PATH(configuration_file));
    std::cout << " done." << std::endl;

    /***				Robot				***/
    std::cout << "[phri::AppMaker] Creating a new robot..." << std::flush;
    auto cp_conf = conf["robot"]["control_point"];
    impl_->robot.emplace(
        spatial::Frame::getAndSave(cp_conf["frame"].as<std::string>()),
        spatial::Frame::getAndSave(cp_conf["parent"].as<std::string>()));
    std::cout << " done." << std::endl;

    std::cout << "[phri::AppMaker] Loading the robot model..." << std::flush;
    impl_->model.emplace(*impl_->robot, conf);
    std::cout << " done." << std::endl;

    std::cout
        << "[phri::AppMaker] Configuring the robot with the model parameters..."
        << std::flush;
    impl_->robot->create(impl_->model->name(), impl_->model->jointCount());
    std::cout << " done." << std::endl;

    /***				Robot driver				***/
    std::cout << "[phri::AppMaker] Creating the robot driver..." << std::flush;
    auto driver_namme = conf["driver"]["type"].as<std::string>();
    impl_->driver = DriverFactory::create(driver_namme, *impl_->robot, conf);
    if (not impl_->driver) {
        throw std::runtime_error(OPEN_PHRI_ERROR(
            "The driver '" + driver_namme +
            "' cannot be created. Make sure its header is included and its "
            "library is linked to your application."));
    }
    impl_->init_timeout = conf["driver"]["init_timeout"].as<double>(30.);
    impl_->start_timeout = conf["driver"]["start_timeout"].as<double>(30.);
    std::cout << " done." << std::endl;

    std::cout << "[phri::AppMaker] Starting the robot driver..." << std::flush;
    impl_->driver->start(impl_->start_timeout);
    std::cout << " done." << std::endl;

    /***			Controller configuration			***/
    std::cout << "[phri::AppMaker] Creating the robot controller..."
              << std::flush;
    impl_->controller.emplace(*impl_->robot, conf);
    std::cout << " done." << std::endl;

    /***			Data logger configuration			***/
    std::cout << "[phri::AppMaker] Creating the data logger..." << std::flush;
    impl_->clock.emplace(impl_->driver->getTimeStep());
    impl_->data_logger.emplace(
        PID_PATH(conf["data_logger"]["folder"].as<std::string>("/tmp")),
        impl_->clock->getTime(), true);

    if (conf["data_logger"]["log_control_data"].as<bool>(false)) {
        impl_->data_logger->logSafetyControllerData(&impl_->controller.value());
    }
    if (conf["data_logger"]["log_robot_data"].as<bool>(false)) {
        impl_->data_logger->logRobotData(&impl_->robot.value());
    }
    std::cout << " done." << std::endl;

    impl_->app_configuration = conf["parameters"];

    impl_->initialized = false;

    std::cout << "[phri::AppMaker] The application is now fully configured."
              << std::endl;
}

AppMaker::~AppMaker() {
    stop();
}

bool AppMaker::init(std::function<bool(void)> init_code) {
    bool all_ok = true;
    std::cout << "[phri::AppMaker] Initializing the robot..." << std::flush;
    all_ok &= impl_->driver->init(impl_->init_timeout);
    impl_->model->forwardKinematics();
    std::cout << " done." << std::endl;
    if (init_code) {
        std::cout << "[phri::AppMaker] Calling user initialization function..."
                  << std::flush;
        all_ok &= init_code();
        std::cout << " done." << std::endl;
    }
    impl_->initialized = all_ok;
    return all_ok;
}

bool AppMaker::run(const callback& pre_controller_code,
                   const callback& post_controller_code) {
    bool ok = true;
    if (not impl_->initialized) {
        std::cerr << "[phri::AppMaker] You must call init() once before run()"
                  << std::endl;
        ok = false;
    } else {
        if (impl_->driver->syncThenRead()) {
            impl_->model->forwardKinematics();
            if (pre_controller_code) {
                ok &= pre_controller_code();
            }
            impl_->controller->compute();
            if (post_controller_code) {
                ok &= post_controller_code();
            }
            if (not impl_->driver->send()) {
                std::cerr << "[phri::AppMaker] Can'send data to the driver"
                          << std::endl;
                ok = false;
            }
            impl_->clock->update();
            impl_->data_logger->process();
        } else {
            std::cerr << "[phri::AppMaker] Can't get data from the driver"
                      << std::endl;
            ok = false;
        }
    }
    return ok;
}

bool AppMaker::operator()(const callback& pre_controller_code,
                          const callback& post_controller_code) {
    return run(pre_controller_code, post_controller_code);
}

bool AppMaker::stop() {
    bool ok = true;
    if (impl_->initialized) {
        std::cout << "[phri::AppMaker] Stopping the robot..." << std::flush;
        ok = impl_->driver->stop();
        std::cout << " done." << std::endl;
        impl_->initialized = false;
    }
    return ok;
}

Robot& AppMaker::robot() {
    return *impl_->robot;
}

SafetyController& AppMaker::controller() {
    return *impl_->controller;
}

RobotModel& AppMaker::model() {
    return *impl_->model;
}

Driver& AppMaker::driver() {
    return *impl_->driver;
}

DataLogger& AppMaker::dataLogger() {
    return *impl_->data_logger;
}

const Robot& AppMaker::robot() const {
    return *impl_->robot;
}

const SafetyController& AppMaker::controller() const {
    return *impl_->controller;
}

const RobotModel& AppMaker::model() const {
    return *impl_->model;
}

const Driver& AppMaker::driver() const {
    return *impl_->driver;
}

const DataLogger& AppMaker::dataLogger() const {
    return *impl_->data_logger;
}

const YAML::Node& AppMaker::getParameters() const {
    return impl_->app_configuration;
}

} // namespace phri
