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
#include <franka/robot_state.h>
#include <yaml-cpp/yaml.h>
#include "protocol/franka_arm_state.hpp"
#include "control_mode/abstract_control_mode.h"

class FrankaProxy {

public:
    // Constructor & Destructor
    explicit FrankaProxy(const std::string& config_path);// Constructor that initializes the proxy with a configuration file
    ~FrankaProxy();// Destructor to clean up resources

    // Core server operations
    bool start();// Starts the Franka server, initializing the robot and communication sockets
    void stop();// Stops the server, cleaning up resources and shutting down communication
    void spin();// Main loop for processing requests
    
    // State management
    franka::RobotState getCurrentState();// Return the current state of the robot

    // Mode management
    void registerControlMode(const std::string& mode, std::unique_ptr<AbstractControlMode> control_mode);//register the map
    void setControlMode(const std::string& mode);
    
    
    
    // Configuration
    void displayConfig() const;
    
private:
    // Initialization
    void initialize(const std::string &filename);// Initializes the FrankaProxy with the given configuration file
    void setupCommunication();// Sets up ZMQ communication sockets and threads?

    // Thread functions
    void statePublishThread();// ZMQ PUB, Publishes the current state of the robot at a fixed rate
    void responseSocketThread();// ZMQ REP,responds to incoming requests from clients
    void controlLoopThread();// Main control loop for processing commands and updating the robot state

    
    // Message handling
    void handleServiceRequest(const std::string& request, std::string& response);
    
    
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
    std::thread state_pub_thread_;//statePublishThread()
    std::thread service_thread_;//responseSocketThread()
    std::thread control_thread_;//controlLoopThread()
    
    
    // Synchronization
    std::atomic<bool> in_control_;//for threads
    
    // State data
    franka::RobotState current_state_;
    
    //Control mode
    std::string mode_name;
    AbstractControlMode* current_mode_ = nullptr;
    std::mutex control_mutex_;
    std::map<std::string, std::shared_ptr<AbstractControlMode>> control_modes_map_;
    
    // TODO: put all the Constants to a config file
    static constexpr int STATE_PUB_RATE_HZ = 100;
    static constexpr int SOCKET_TIMEOUT_MS = 100;
    static constexpr int MAX_MESSAGE_SIZE = 4096;
};

#endif // FRANKA_SERVER_HPP