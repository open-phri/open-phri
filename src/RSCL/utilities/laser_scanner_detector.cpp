#include <RSCL/utilities/laser_scanner_detector.h>
#include <iostream>

using namespace RSCL;

LaserScannerDetector::LaserScannerDetector(
	VectorXdConstPtr laser_data,
	double scanning_angle,
	double minimum_distance,
	double maximum_distance,
	double threshold) :
	laser_data_(laser_data),
	scanning_angle_(scanning_angle),
	minimum_distance_(minimum_distance),
	maximum_distance_(maximum_distance),
	threshold_(threshold)
{
	position_ = std::make_shared<Vector2d>();
	distance_ = std::make_shared<double>();
}

Vector2dConstPtr LaserScannerDetector::getPosition() const {
	return position_;
}

doubleConstPtr LaserScannerDetector::getDistance() const {
	return distance_;
}

void LaserScannerDetector::init() {
	maximum_distances_.resize(laser_data_->size());
	const auto& dist = *laser_data_;
	for (size_t i = 0; i < laser_data_->size(); ++i) {
		maximum_distances_(i) = dist(i) < maximum_distance_ && dist(i) > minimum_distance_ ? dist(i) : maximum_distance_;
	}
}

double LaserScannerDetector::compute() {
	double step_size = scanning_angle_ / laser_data_->size();
	int min_dist_idx = -1;
	const auto& dist = *laser_data_;
	double min_dist = maximum_distance_;

	for (size_t i = 0; i < dist.size(); ++i) {
		double di = dist(i);
		if((di < maximum_distances_(i) - threshold_) && (di >= minimum_distance_) && (di < min_dist)) {
			min_dist_idx = i;
			min_dist = di;
		}
	}

	if(min_dist_idx >= 0) {
		double angle = (min_dist_idx - laser_data_->size()/2) * step_size;
		position_->x() = min_dist * std::cos(angle);
		position_->y() = min_dist * std::sin(angle);
	}
	else {
		position_->x() = maximum_distance_;
		position_->y() = 0.;
	}

	// std::cout << "position: " << position_->transpose() << ", min_dist: " << min_dist << std::endl;

	*distance_ = min_dist;
	return min_dist;
}

double LaserScannerDetector::operator()() {
	return compute();
}
