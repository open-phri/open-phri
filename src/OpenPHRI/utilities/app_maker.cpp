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
};

AppMaker::AppMaker(const std::string& configuration_file) :
	impl_(std::make_unique<AppMaker::pImpl>())
{
	auto conf = YAML::LoadFile(PID_PATH(configuration_file));
	/***				Robot				***/
	impl_->robot = std::make_shared<Robot>();

	impl_->model = std::make_shared<RobotModel>(
		impl_->robot,
		conf);

	impl_->robot->create(impl_->model->name(), impl_->model->jointCount());

	/***				V-REP driver				***/
	impl_->driver = DriverFactory::create(
		conf["driver"]["type"].as<std::string>(),
		impl_->robot,
		conf);

	impl_->driver->start();

	/***			Controller configuration			***/
	impl_->controller = std::make_shared<SafetyController>(impl_->robot, conf);

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

	impl_->app_configuration = conf["parameters"];
}

AppMaker::~AppMaker() = default;

bool AppMaker::init(std::function<bool(void)> init_code) {
	bool all_ok = true;
	all_ok &= impl_->driver->init();
	if(init_code) {
		all_ok &= init_code();
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
	return impl_->driver->stop();
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
