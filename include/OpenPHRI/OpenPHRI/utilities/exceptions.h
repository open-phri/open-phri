#pragma once

#include <exception>
#include <string>

namespace phri {

// Original solution found at
// http://stackoverflow.com/questions/37181621/easy-way-of-constructing-information-message-for-throwing-stdexception-using-p
[[nodiscard]] inline std::string make_open_phri_error(const std::string& msg,
                                                      char const* file,
                                                      char const* function,
                                                      std::size_t line) {
    using namespace std::string_literals;
    return "OpenPHRI error in "s + file + ":" + std::to_string(line) + " [" +
           function + "]: " + msg;
}

[[nodiscard]] inline std::string make_open_phri_warning(const std::string& msg,
                                                        char const* function) {
    using namespace std::string_literals;
    return "["s + function + "]: "s + msg;
}

#define OPEN_PHRI_ERROR(...)                                                   \
    phri::make_open_phri_error(__VA_ARGS__, __FILE__, __PRETTY_FUNCTION__,     \
                               __LINE__)

#define OPEN_PHRI_WARNING(...)                                                 \
    phri::make_open_phri_warning(__VA_ARGS__, __PRETTY_FUNCTION__)

} // namespace phri
