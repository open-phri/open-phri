#pragma once

#include <RSCL/definitions.h>

namespace RSCL {

template<typename T>
class Integrator {
public:
	/***		Constructor & destructor		***/
	Integrator(std::shared_ptr<const T> input, double sample_time) :
		input_(input), sample_time_(sample_time)
	{
		output_ = std::make_shared<T>();

		reset();
	}

	virtual ~Integrator() = default;

	/***		Configuration		***/
	std::shared_ptr<const T> getOutput() {
		return output_;
	}

	void reset() {
		*output_ = *input_;
	}

	void force(std::shared_ptr<const T> value) {
		*output_ = *value;
	}

	void force(const T& value) {
		*output_ = value;
	}

	/***		Algorithm		***/
	virtual T compute() {
		*output_ += *input_ * sample_time_;
		return *output_;
	}

private:
	std::shared_ptr<const T> input_;
	std::shared_ptr<T> output_;
	double sample_time_;
};

template<typename T>
using IntegratorPtr = std::shared_ptr<Integrator<T>>;
template<typename T>
using IntegratorConstPtr = std::shared_ptr<const Integrator<T>>;

extern template class Integrator<double>;
extern template class Integrator<Vector2d>;
extern template class Integrator<Vector3d>;
extern template class Integrator<Vector6d>;

} // namespace RSCL
