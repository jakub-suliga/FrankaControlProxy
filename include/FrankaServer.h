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

#include "FrankaArmState.h"

class FrankaServer {
public:
    enum class Mode {
        IDLE,
        POSITION_CONTROL,
        VELOCITY_CONTROL,
        TORQUE_CONTROL,
        IMPEDANCE_CONTROL,
        MANUAL_GUIDING
    };

public:
    // Constructor & Destructor
    explicit FrankaServer(const std::string& config_path);
    ~FrankaServer();

    // Core server operations
    bool start();
    void stop();
    void spin();
    
    // State management
    bool isRunning() const;
    void updateRobotState(const FrankaArmState& state);
    FrankaArmState getCurrentState() const;
    
    // Mode management
    void setMode(Mode mode);
    Mode getCurrentMode() const;
    
    // Configuration
    void loadConfig(const std::string& config_path);
    void displayConfig() const;

private:
    // Initialization
    void initializeRobot();
    void initializeSockets();
    void setRobotSafetyParameters();
    
    // Thread functions
    void robotStatePubThread();
    void responseSocketThread();
    void commandSubscriberThread();
    
    // Message handling
    void handleServiceRequest(const std::string& request, std::string& response);
    void handleCommandMessage(const std::string& command);
    
    // State conversion
    FrankaArmState convertFrankaState(const franka::RobotState& franka_state) const;
    
    // Utility functions
    std::string createStateMessage(const FrankaArmState& state) const;
    std::string createModeMessage(Mode mode) const;
    Mode parseModeFromString(const std::string& mode_str) const;
    
    // Thread management
    void startThreads();
    void stopThreads();
    void joinThreads();

private:
    // Configuration
    std::string robot_ip_;
    std::string state_pub_addr_;
    std::string service_addr_;
    
    // Franka robot
    std::unique_ptr<franka::Robot> robot_;
    std::unique_ptr<franka::Model> model_;
    
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
    Mode current_mode_;
    
    // Constants
    static constexpr int STATE_PUB_RATE_HZ = 100;
    static constexpr int SOCKET_TIMEOUT_MS = 100;
    static constexpr int MAX_MESSAGE_SIZE = 4096;
};

#endif // FRANKA_SERVER_HPP