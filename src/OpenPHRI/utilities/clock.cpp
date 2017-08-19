#include <OpenPHRI/utilities/clock.h>

using namespace OpenPHRI;

Clock::Clock() : sample_time_(-1.) {
	reset();
	time_ = std::make_shared<double>(0.);
}

Clock::Clock(double sample_time) : sample_time_(sample_time) {
	time_ = std::make_shared<double>(0.);
}

void Clock::reset() {
	if(sample_time_ < 0) {
		init_time_ = std::chrono::high_resolution_clock::now();
	}
	else {
		*time_ = 0.;
	}
}

std::shared_ptr<double> Clock::getTime() const {
	return time_;
}

double Clock::update() {
	auto& current_time = *time_;
	if(sample_time_ < 0) {
		current_time = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - init_time_).count();
	}
	else {
		current_time += sample_time_;
	}
	return current_time;
}

double Clock::operator()() {
	return update();
}
