#pragma once

#include <OpenPHRI/robot.h>
#include <OpenPHRI/fwd_decl.h>
#include <functional>
#include <map>

namespace phri {

class Driver {
public:
    Driver(Robot& robot, double sample_time);

    virtual ~Driver();

    /**
     * @brief Initialize the communication with the robot.
     * @details Must be called by derived classes on successfull initialization
     * when the robot state has correctly been updated.
     * @param timeout The maximum time to wait to establish the connection.
     * @return true on success, false otherwise
     */
    virtual bool init(double timeout = 30.);

    /**
     * Start the communication with the robot.
     * @return true on success, false otherwise
     */
    virtual bool start(double timeout = 30.) = 0;

    /**
     * Stop the communication with the robot.
     * @return true on success, false otherwise
     */
    virtual bool stop() = 0;

    /**
     * Get data from the robot (current state)
     * @return true on success, false otherwise
     */
    virtual bool read() = 0;

    /**
     * Send data to the robot (commands)
     * @return true on success, false otherwise
     */
    virtual bool send() = 0;

    virtual bool sync();

    virtual double getSampleTime() const final;

protected:
    Robot& robot_;
};

class DriverFactory {
public:
    using create_method_t =
        std::function<std::shared_ptr<Driver>(Robot&, const YAML::Node&)>;

    DriverFactory() = default;
    ~DriverFactory() = default;

    static bool add(const std::string name, create_method_t create_method) {
        auto it = createMethods().find(name);
        if (it == createMethods().end()) {
            createMethods()[name] = create_method;
            return true;
        }
        return false;
    }

    template <typename T> static bool add(const std::string name) {
        auto it = createMethods().find(name);
        if (it == createMethods().end()) {
            createMethods()[name] =
                [](Robot& robot,
                   const YAML::Node& conf) -> std::shared_ptr<Driver> {
                return std::make_shared<T>(robot, conf);
            };
            return true;
        }
        return false;
    }

    static std::shared_ptr<Driver> create(std::string name, Robot& robot,
                                          const YAML::Node& configuration) {
        auto it = createMethods().find(name);
        if (it != createMethods().end()) {
            return it->second(robot, configuration);
        }

        return nullptr;
    }

private:
    static std::map<std::string, create_method_t>& createMethods() {
        static std::map<std::string, create_method_t> create_methods;
        return create_methods;
    }
};

using DriverPtr = std::shared_ptr<Driver>;
using DriverConstPtr = std::shared_ptr<const Driver>;

} // namespace phri
