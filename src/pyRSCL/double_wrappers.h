#pragma once

#include <memory>

// Primitive types need to be wrapped in order to use pointers on them in python
struct DoubleWrap {
	DoubleWrap(double init_value = 0.) :
		value(std::make_shared<double>(init_value))
	{
	}

	DoubleWrap(std::shared_ptr<double> ptr) :
		value(ptr)
	{
	}

	operator std::shared_ptr<double>() const {
		return value;
	}

	operator std::shared_ptr<const double>() const {
		return value;
	}

	void set(double val) {
		*value = val;
	}

	double get() const {
		return *value;
	}

private:
	std::shared_ptr<double> value;
};

struct ConstDoubleWrap {
	ConstDoubleWrap(std::shared_ptr<const double> ptr) :
		value(ptr)
	{
	}

	operator std::shared_ptr<const double>() const {
		return value;
	}

	double get() const {
		return *value;
	}

private:
	std::shared_ptr<const double> value;
};
