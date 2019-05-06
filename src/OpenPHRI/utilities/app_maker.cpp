#include <OpenPHRI/OpenPHRI.h>

#include <pid/rpath.h>

using namespace phri;

struct AppMaker::pImpl {
    RobotPtr robot;
    SafetyControllerPtr controller;
    RobotModelPtr model;
    DriverPtr driver;
    DataLoggerPtr data_logger;
    ClockPtr clock;
    YAML::Node app_configuration;
    double init_timeout;
    double start_timeout;
};

AppMaker::AppMaker(const std::string& configuration_file)
    : impl_(std::make_unique<AppMaker::pImpl>()) {
    std::cout << "[phri::AppMaker] Parsing the configuration file..."
              << std::flush;
    auto conf = YAML::LoadFile(PID_PATH(configuration_file));
    std::cout << " done." << std::endl;

    /***				Robot				***/
    std::cout << "[phri::AppMaker] Creating a new robot..." << std::flush;
    impl_->robot = std::make_shared<Robot>();
    std::cout << " done." << std::endl;

    std::cout << "[phri::AppMaker] Loading the robot model..." << std::flush;
    impl_->model = std::make_shared<RobotModel>(*impl_->robot, conf);
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
    impl_->controller = std::make_shared<SafetyController>(*impl_->robot, conf);
    std::cout << " done." << std::endl;

    /***			Data logger configuration			***/
    std::cout << "[phri::AppMaker] Creating the data logger..." << std::flush;
    impl_->clock = std::make_shared<Clock>(impl_->driver->getSampleTime());
    impl_->data_logger = std::make_shared<DataLogger>(
        PID_PATH(conf["data_logger"]["folder"].as<std::string>("/tmp")),
        impl_->clock->getTime(), true);

    if (conf["data_logger"]["log_control_data"].as<bool>(false)) {
        impl_->data_logger->logSafetyControllerData(impl_->controller.get());
    }
    if (conf["data_logger"]["log_robot_data"].as<bool>(false)) {
        impl_->data_logger->logRobotData(impl_->robot);
    }
    std::cout << " done." << std::endl;

    impl_->app_configuration = conf["parameters"];

    std::cout << "[phri::AppMaker] The application is now fully configured."
              << std::endl;
}

AppMaker::~AppMaker() = default;

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
    return all_ok;
}

bool AppMaker::run(std::function<bool(void)> pre_controller_code,
                   std::function<bool(void)> post_controller_code) {
    bool ok = true;
    if (impl_->driver->read()) {
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
    return ok;
}

bool AppMaker::stop() {
    std::cout << "[phri::AppMaker] Stopping the robot..." << std::flush;
    bool ok = impl_->driver->stop();
    std::cout << " done." << std::endl;
    return ok;
}

Robot& AppMaker::robot() {
    return *impl_->robot;
}

RobotPtr AppMaker::robotPtr() const {
    return impl_->robot;
}

SafetyController& AppMaker::controller() {
    return *impl_->controller;
}

SafetyControllerPtr AppMaker::controllerPtr() const {
    return impl_->controller;
}

RobotModel& AppMaker::model() {
    return *impl_->model;
}

RobotModelPtr AppMaker::modelPtr() const {
    return impl_->model;
}

Driver& AppMaker::driver() {
    return *impl_->driver;
}

DriverPtr AppMaker::driverPtr() const {
    return impl_->driver;
}

DataLogger& AppMaker::dataLogger() {
    return *impl_->data_logger;
}

DataLoggerPtr AppMaker::dataLoggerPtr() const {
    return impl_->data_logger;
}

const YAML::Node& AppMaker::getParameters() const {
    return impl_->app_configuration;
}
