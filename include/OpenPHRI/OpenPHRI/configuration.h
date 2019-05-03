#pragma once

#include <OpenPHRI/fwd_decl.h>

namespace phri {

//! \brief Configurations can be created to hold multiple generators and
//! constraints together. They allow for a quick configuration of a
//! SafetyController with common working modes (e.g ISO15066).
//!
// TODO
class Configuration {
public:
private:
    friend SafetyController;
    void configure(SafetyController& controller);
};

} // namespace phri