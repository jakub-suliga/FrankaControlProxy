#include <iostream>
#include <fstream>
#include <string>
#include <csignal>
#include <functional>
#include <mutex>
#include <thread>
#include <atomic>
#include <zmq.hpp>
#include <yaml-cpp/yaml.h>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>

#include "FrankaProxy.h"
#include "FrankaState.h"
#include "control_mode/AbstractControlMode.h"
#include "control_mode/ZeroTorqueControlMode.h"


int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "Please input the config path" << std::endl;
        return 0;
    }
    
    FrankaProxy proxy(argv[1]);
    proxy.registerControlMode("ZeroTorque", std::make_unique<ZeroTorqueControlMode>());
    proxy.start();
    proxy.displayConfig();
    try {
        proxy.spin();
    } catch (const std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}