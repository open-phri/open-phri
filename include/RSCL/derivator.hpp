#pragma once

#include <RSCL/definitions.h>

namespace RSCL {

template<typename T>
class Derivator {
public:
	/***		Constructor & destructor		***/
	Derivator(std::shared_ptr<const T> input, double sample_time) :
		input_(input), frequency_(1./sample_time)
	{
		output_ = std::make_shared<T>();

		reset();
	}

	virtual ~Derivator() = default;

	/***		Configuration		***/
	std::shared_ptr<const T> getOutput() {
		return output_;
	}

	template<typename TT = T>
	typename std::enable_if<std::is_same<TT, Eigen::Matrix<typename TT::Scalar,
	                                                       TT::RowsAtCompileTime, TT::ColsAtCompileTime,
	                                                       TT::Options, TT::MaxRowsAtCompileTime,
	                                                       TT::MaxColsAtCompileTime>>::value, void>::type
	reset() {
		output_->setZero();
		previous_input_ = *input_;
	}

	template<typename TT = T>
	typename std::enable_if<std::is_arithmetic<TT>::value, void>::type
	reset() {
		*output_ = TT(0);
		previous_input_ = *input_;
	}

	/***		Algorithm		***/
	virtual T compute() {
		*output_ = (*input_ - previous_input_) * frequency_;
		previous_input_ = *input_;
		return *output_;
	}

private:
	std::shared_ptr<const T> input_;
	std::shared_ptr<T> output_;
	T previous_input_;
	double frequency_;
};

template<typename T>
using DerivatorPtr = std::shared_ptr<Derivator<T>>;
template<typename T>
using DerivatorConstPtr = std::shared_ptr<const Derivator<T>>;

extern template class Derivator<double>;
extern template class Derivator<Vector2d>;
extern template class Derivator<Vector3d>;
extern template class Derivator<Vector6d>;

} // namespace RSCL
