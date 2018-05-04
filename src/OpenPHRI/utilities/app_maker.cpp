#include <OpenPHRI/utilities/app_maker.h>

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
};

AppMaker::AppMaker(const std::string& configuration_file) :
	impl_(std::make_unique<AppMaker::pImpl>())
{
	std::cout << "[OpenPHRI::AppMaker] Parsing the configuration file...";
	auto conf = YAML::LoadFile(PID_PATH(configuration_file));
	std::cout << " done.\n";

	/***				Robot				***/
	std::cout << "[OpenPHRI::AppMaker] Creating a new robot...";
	impl_->robot = std::make_shared<Robot>();
	std::cout << " done.\n";

	std::cout << "[OpenPHRI::AppMaker] Loading the robot model...";
	impl_->model = std::make_shared<RobotModel>(
		impl_->robot,
		conf);
	std::cout << " done.\n";

	std::cout << "[OpenPHRI::AppMaker] Configuring the robot with the model parameters...";
	impl_->robot->create(impl_->model->name(), impl_->model->jointCount());
	std::cout << " done.\n";

	/***				Robot driver				***/
	std::cout << "[OpenPHRI::AppMaker] Creating the robot driver...";
	impl_->driver = DriverFactory::create(
		conf["driver"]["type"].as<std::string>(),
		impl_->robot,
		conf);
	impl_->init_timeout = conf["driver"]["init_timeout"].as<double>(30.);
	std::cout << " done.\n";

	std::cout << "[OpenPHRI::AppMaker] Starting the robot driver...";
	impl_->driver->start();
	std::cout << " done.\n";

	/***			Controller configuration			***/
	std::cout << "[OpenPHRI::AppMaker] Creating the robot controller...";
	impl_->controller = std::make_shared<SafetyController>(impl_->robot, conf);
	std::cout << " done.\n";

	/***			Data logger configuration			***/
	std::cout << "[OpenPHRI::AppMaker] Creating the data logger...";
	impl_->clock = std::make_shared<Clock>(impl_->driver->getSampleTime());
	impl_->data_logger = std::make_shared<DataLogger>(
		PID_PATH(conf["data_logger"]["folder"].as<std::string>("/tmp")),
		impl_->clock->getTime(),
		true);

	if(conf["data_logger"]["log_control_data"].as<bool>(false)) {
		impl_->data_logger->logSafetyControllerData(impl_->controller.get());
	}
	if(conf["data_logger"]["log_robot_data"].as<bool>(false)) {
		impl_->data_logger->logRobotData(impl_->robot);
	}
	std::cout << " done.\n";

	impl_->app_configuration = conf["parameters"];

	std::cout << "[OpenPHRI::AppMaker] The application is now fully configured.";
}

AppMaker::~AppMaker() = default;

bool AppMaker::init(std::function<bool(void)> init_code) {
	bool all_ok = true;
	std::cout << "[OpenPHRI::AppMaker] Initializing the robot...";
	all_ok &= impl_->driver->init(impl_->init_timeout);
	std::cout << " done.\n";
	if(init_code) {
		std::cout << "[OpenPHRI::AppMaker] Calling user initialization function...";
		all_ok &= init_code();
		std::cout << " done.\n";
	}
	return all_ok;
}

bool AppMaker::run(
	std::function<bool(void)> pre_controller_code,
	std::function<bool(void)> post_controller_code)
{
	bool ok = true;
	if(impl_->driver->read()) {
		impl_->model->forwardKinematics();
		if(pre_controller_code) {
			pre_controller_code();
		}
		impl_->controller->compute();
		if(post_controller_code) {
			post_controller_code();
		}
		if(not impl_->driver->send()) {
			std::cerr << "[OpenPHRI::AppMaker] Can'send data to the driver" << std::endl;
			ok = false;
		}
		impl_->clock->update();
		impl_->data_logger->process();
	}
	else {
		std::cerr << "[OpenPHRI::AppMaker] Can't get data from the driver" << std::endl;
		ok = false;
	}
	return ok;
}

bool AppMaker::stop() {
	std::cout << "[OpenPHRI::AppMaker] Stopping the robot...";
	bool ok = impl_->driver->stop();
	std::cout << " done.\n";
	return ok;
}

RobotPtr AppMaker::getRobot() const {
	return impl_->robot;
}

SafetyControllerPtr AppMaker::getController() const {
	return impl_->controller;
}

RobotModelPtr AppMaker::getModel() const {
	return impl_->model;
}

DriverPtr AppMaker::getDriver() const {
	return impl_->driver;
}

DataLoggerPtr AppMaker::getDataLogger() const {
	return impl_->data_logger;
}

const YAML::Node& AppMaker::getParameters() const {
	return impl_->app_configuration;
}
