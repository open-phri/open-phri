#include <OpenPHRI/utilities/app_maker.h>

#include <pid/rpath.h>
#include <yaml-cpp/yaml.h>

using namespace phri;

AppMaker::AppMaker(const std::string& configuration_file) {
	auto conf = YAML::LoadFile(PID_PATH(configuration_file));

	/***				Robot				***/
	robot_ = std::make_shared<Robot>();

	model_ = std::make_shared<RobotModel>(
		robot_,
		conf);

	robot_->create(model_->name(), model_->jointCount());

	/***				V-REP driver				***/
	driver_ = DriverFactory::create(
		conf["driver"]["type"].as<std::string>(),
		robot_,
		conf);

	driver_->start();

	/***			Controller configuration			***/
	controller_ = std::make_shared<SafetyController>(robot_, conf);

	clock_ = std::make_shared<Clock>(driver_->getSampleTime());
	data_logger_ = std::make_shared<DataLogger>(
		"/tmp",
		clock_->getTime(),
		true);

	data_logger_->logSafetyControllerData(controller_.get());
	data_logger_->logRobotData(robot_);
}
AppMaker::~AppMaker() = default;

bool AppMaker::init(std::function<bool(void)> init_code) {
	bool all_ok = true;
	all_ok &= driver_->init();
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
	if(driver_->read()) {
		model_->forwardKinematics();
		if(pre_controller_code) {
			pre_controller_code();
		}
		controller_->compute();
		if(post_controller_code) {
			post_controller_code();
		}
		if(not driver_->send()) {
			std::cerr << "[OpenPHRI::AppMaker] Can'send data to the driver" << std::endl;
			ok = false;
		}
		clock_->update();
		data_logger_->process();
	}
	else {
		std::cerr << "[OpenPHRI::AppMaker] Can't get data from the driver" << std::endl;
		ok = false;
	}
	return ok;
}

bool AppMaker::stop() {
	return driver_->stop();
}

RobotPtr AppMaker::getRobot() const {
	return robot_;
}

SafetyControllerPtr AppMaker::getController() const {
	return controller_;
}

RobotModelPtr AppMaker::getModel() const {
	return model_;
}

DriverPtr AppMaker::getDriver() const {
	return driver_;
}

DataLoggerPtr AppMaker::getDataLogger() const {
	return data_logger_;
}
