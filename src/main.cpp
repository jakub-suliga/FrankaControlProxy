#include "FrankaProxy.hpp"
#include "control_mode_registry.h"
#include <iostream>


int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "Please input the config path" << std::endl;
        return 0;
    }

    std::string config_path = argv[1];
    FrankaProxy proxy(config_path);
    registerAllControlModes(proxy);
    proxy.setControlMode("idle");
    proxy.start();
    proxy.spin();

    return 0;
}
