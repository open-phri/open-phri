#pragma once

#include <exception>
#include <string>

namespace phri {

// Original solution found at
// http://stackoverflow.com/questions/37181621/easy-way-of-constructing-information-message-for-throwing-stdexception-using-p
inline std::string make_open_phri_error(const std::string& msg,
                                        char const* file, char const* function,
                                        std::size_t line) {
    return std::string{"OpenPHRI error in "} + file + ":" +
           std::to_string(line) + " [" + function + "]: " + msg;
}
#define OPEN_PHRI_ERROR(...)                                                   \
    phri::make_open_phri_error(__VA_ARGS__, __FILE__, __PRETTY_FUNCTION__,     \
                               __LINE__)

} // namespace phri
