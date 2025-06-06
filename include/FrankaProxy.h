#ifndef FRANKA_SERVER_HPP
#define FRANKA_SERVER_HPP

#include <zmq.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <memory>

#include <franka/robot.h>
#include <franka/model.h>
#include <yaml-cpp/yaml.h>

#include "FrankaState.h"
#include "control_mode/AbstractControlMode.h"

class FrankaProxy {

public:
    // Constructor & Destructor
    explicit FrankaProxy(const std::string& config_path);
    ~FrankaProxy();

    // Core server operations
    bool start();
    void stop();
    void spin();
    
    // State management
    const FrankaArmState& getCurrentState() const;
    
    // Mode management
    void setControlMode(const std::string& mode);
    
    // Configuration
    void registerControlMode(const std::string& mode, std::unique_ptr<AbstractControlMode> control_mode);
    void displayConfig() const;

private:
    // Initialization
    void initialize(const std::string &filename);
    
    // Thread functions
    void statePublishThread();
    void responseSocketThread();
    
    // Message handling
    void handleServiceRequest(const std::string& request, std::string& response);
    void handleCommandMessage(const std::string& command);
    
    // State conversion
    FrankaArmState convertFrankaState(const franka::RobotState& franka_state) const;
    
    // Utility functions
    std::string createStateMessage(const FrankaArmState& state) const;
    std::string createModeMessage(const std::string& mode) const;

private:
    // Configuration
    YAML::Node proxy_config_;
    std::string robot_ip_;
    std::string state_pub_addr_;
    std::string service_addr_;
    
    // Franka robot
    std::shared_ptr<franka::Robot> robot_;
    std::shared_ptr<franka::Model> model_;
    
    // ZMQ communication
    zmq::context_t context_;
    zmq::socket_t pub_socket_;
    zmq::socket_t sub_socket_;
    zmq::socket_t rep_socket_;
    
    // Threading
    std::thread state_pub_thread_;
    std::thread control_thread_;
    std::thread service_thread_;
    
    // Synchronization
    std::atomic<bool> in_control_;
    
    // State data
    FrankaArmState current_state_;
    std::string current_mode_;
    std::map<std::string, std::shared_ptr<AbstractControlMode>> control_modes_map_;
    
    // TODO: put all the Constants to a config file
    static constexpr int STATE_PUB_RATE_HZ = 100;
    static constexpr int SOCKET_TIMEOUT_MS = 100;
    static constexpr int MAX_MESSAGE_SIZE = 4096;
};

#endif // FRANKA_SERVER_HPP