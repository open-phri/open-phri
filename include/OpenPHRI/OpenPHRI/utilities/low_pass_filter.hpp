/**
 * @file low_pass_filter.h
 * @author Benjamin Navarro
 * @brief Definition of the LowPassFilter class
 * @date April 2019
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/type_aliases.h>
#include <cassert>
#include <cmath>
#include <iostream>

#include <physical_quantities/spatial/type_aliases.h>

namespace phri {

/** @brief A simple low pass filter
 */
template <typename T> class LowPassFilter {
public:
    using input_type = std::shared_ptr<const T>;
    using output_type = std::shared_ptr<T>;

    LowPassFilter(input_type input, output_type output,
                  double coefficient = 0.5)
        : input_(input), output_(output) {
        setCoefficient(coefficient);
    }

    LowPassFilter(input_type input, output_type output, double sample_frequency,
                  double cutoff_frequency)
        : input_(input), output_(output) {
        setCoefficient(sample_frequency, cutoff_frequency);
    }

    virtual ~LowPassFilter() = default;

    void setCoefficient(double coefficient) {
        assert(coefficient >= 0.);
        assert(coefficient <= 1.);

        coefficient_ = coefficient;
    }

    void setCoefficient(double sample_frequency, double cutoff_frequency) {
        double sample_time = 1. / sample_frequency;
        double time_constant = 1. / (2. * M_PI * cutoff_frequency);
        assert(sample_time > 0.);
        assert(time_constant > 0.);

        if (not(time_constant > 5. * sample_time)) {
            std::cout
                << "[phri::LowPassFilter::setCoefficient] the time constant ("
                << time_constant
                << ") for the low pass filter should be at least five times "
                   "greater than the sample time ("
                << sample_time << ") to have a correct behavior" << std::endl;
        }

        setCoefficient(sample_time / (time_constant + sample_time));
    }

    double getCoefficient() const {
        return coefficient_;
    }

    void reset() {
        previous_input_ = *input_;
        *output_ = *input_;
    }

    /**
     * @brief Update the filter output.
     */
    virtual void compute() {
        *output_ = *input_ * getCoefficient() +
                   previous_input_ * (1. - getCoefficient());
        previous_input_ = *input_;
    }

    /**
     * @brief Call operator, shortcut for compute()
     */
    void operator()() {
        compute();
    }

private:
    input_type input_;
    T previous_input_;
    output_type output_;
    double coefficient_;
};

extern template class LowPassFilter<double>;
extern template class LowPassFilter<Eigen::Vector2d>;
extern template class LowPassFilter<Eigen::Vector3d>;
extern template class LowPassFilter<Eigen::Vector6d>;
extern template class LowPassFilter<Eigen::VectorXd>;

} // namespace phri
