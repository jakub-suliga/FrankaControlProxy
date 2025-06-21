#include "control_mode/control_mode_registry.h"
#include "control_mode/idle_control_mode.hpp"


void registerAllControlModes(FrankaProxy& proxy) {
    proxy.registerControlMode("idle", std::make_unique<IdleControlMode>());
   
}