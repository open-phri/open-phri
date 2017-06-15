#include <RSCL/utilities/clock.h>

using namespace RSCL;

Clock::Clock() : sample_time_(-1.) {
	init_time_ = std::chrono::high_resolution_clock::now();
	time_ = std::make_shared<double>(0.);
}

Clock::Clock(double sample_time) : sample_time_(sample_time) {
	time_ = std::make_shared<double>(0.);
}

std::shared_ptr<double> Clock::getTime() const {
	return time_;
}

void Clock::update() {
	if(sample_time_ < 0) {
		*time_ = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - init_time_).count();
	}
	else {
		*time_ += sample_time_;
	}
}
void Clock::operator()() {
	update();
}
