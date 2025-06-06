#include "FrankaProxy.h"
#include <iostream>
#include <sstream>
#include <chrono>
#include <exception>

// Constructor & Destructor
FrankaProxy::FrankaProxy(const std::string& config_path)
    : context_(1)
    , pub_socket_(context_, ZMQ_PUB)
    , sub_socket_(context_, ZMQ_SUB)
    , rep_socket_(context_, ZMQ_REP)
    , in_control_(false)
    , current_mode_("ZeroTorque") // Default control mode
{
    try {
        YAML::Node proxy_config_ = YAML::LoadFile(config_path);
        // TODO： change the key names to be more descriptive
        robot_ip_ = proxy_config_["robot"]["ip"].as<std::string>();
        state_pub_addr_ = proxy_config_["communication"]["state_pub_addr"].as<std::string>();
        service_addr_ = proxy_config_["communication"]["service_addr"].as<std::string>();
        
        std::cout << "Configuration loaded from: " << config_path << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to load configuration: " << e.what() << std::endl;
        throw;
    }

    robot_ = std::make_unique<franka::Robot>(proxy_config_["robot"]["ip"].as<std::string>());
    // NOTE: default setting, and each control mode should set its own collision behavior
    robot_->setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    robot_->setJointImpedance({{0, 0, 0, 0, 0, 0, 0}});
    model_ = std::make_unique<franka::Model>(robot_->loadModel());

}

FrankaProxy::~FrankaProxy() {
    stop();
}

// Core server operations
bool FrankaProxy::start() {
    try {
        // TODO: Implement server startup logic
        // - Bind ZMQ sockets
        // - Start communication threads
        // - Initialize robot connection
        
        std::cout << "Starting FrankaProxy server..." << std::endl;
        
        // Bind sockets
        pub_socket_.bind(state_pub_addr_);
        rep_socket_.bind(service_addr_);
        
        // Set socket options
        int timeout = SOCKET_TIMEOUT_MS;
        pub_socket_.set(zmq::sockopt::sndtimeo, timeout);
        rep_socket_.set(zmq::sockopt::rcvtimeo, timeout);
        rep_socket_.set(zmq::sockopt::sndtimeo, timeout);
        
        // Start threads
        state_pub_thread_ = std::thread(&FrankaProxy::statePublishThread, this);
        service_thread_ = std::thread(&FrankaProxy::responseSocketThread, this);
        
        in_control_.store(true);
        
        std::cout << "FrankaProxy server started successfully" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to start server: " << e.what() << std::endl;
        return false;
    }
}

void FrankaProxy::stop() {
    std::cout << "Stopping FrankaProxy server..." << std::endl;
    
    in_control_.store(false);
    
    // Join threads
    if (state_pub_thread_.joinable()) {
        state_pub_thread_.join();
    }
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
    if (service_thread_.joinable()) {
        service_thread_.join();
    }
    
    // Close sockets
    pub_socket_.close();
    sub_socket_.close();
    rep_socket_.close();
    
    std::cout << "FrankaProxy server stopped" << std::endl;
}

void FrankaProxy::statePublishThread() {
    auto last_time = std::chrono::high_resolution_clock::now();
    const auto period = std::chrono::microseconds(1000000 / STATE_PUB_RATE_HZ);
    
    while (in_control_.load()) {
        try {
            // TODO: Implement main spin loop
            // - Read robot state from Franka
            // - Update current_state_
            // - Publish state message
            
            auto current_time = std::chrono::high_resolution_clock::now();
            
            if (robot_) {                
                // Publish state
                char encoded_state = current_state_.encode();
                pub_socket_.send(zmq::buffer(&encoded_state, sizeof(encoded_state)), zmq::send_flags::none);
            }
            
            // Sleep to maintain loop rate
            std::this_thread::sleep_until(last_time + period);
            last_time += period;
            
        } catch (const std::exception& e) {
            std::cerr << "Error in spin loop: " << e.what() << std::endl;
        }
    }
}

const FrankaArmState& FrankaProxy::getCurrentState() const {
    // TODO: Implement thread-safe state retrieval
    // - Return copy of current_state_
    
    return current_state_;
}

// Mode management
void FrankaProxy::setControlMode(const std::string& mode) {
    // TODO: Implement mode switching logic
    // - Validate mode transition
    // - Update current_mode_
    // - Notify relevant components
    
    std::cout << "Switching to mode " << mode << std::endl;
    current_mode_ = mode;
}


void FrankaProxy::displayConfig() const {
    // TODO: Implement configuration display
    // - Print current configuration values
    
    std::cout << "=== FrankaProxy Configuration ===" << std::endl;
    std::cout << "Robot IP: " << robot_ip_ << std::endl;
    std::cout << "State Publisher Address: " << state_pub_addr_ << std::endl;
    std::cout << "Service Address: " << service_addr_ << std::endl;
    std::cout << "=================================" << std::endl;
}

// Private methods
void FrankaProxy::initialize(const std::string& filename) {
    try {
        // TODO: Implement initialization logic
        // - Create robot and model instances
        // - Setup initial robot configuration
        // - Initialize state structures
        
        std::cout << "Initializing robot connection to: " << robot_ip_ << std::endl;
        
        robot_ = std::make_unique<franka::Robot>(robot_ip_);
        model_ = std::make_unique<franka::Model>(robot_->loadModel());
        
        // Set default collision behavior
        robot_->setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}
        );
        
        std::cout << "Robot initialized successfully" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize robot: " << e.what() << std::endl;
        throw;
    }
}

// Message handling
void FrankaProxy::handleServiceRequest(const std::string& request, std::string& response) {
    // TODO: Implement service request handling
    // - Parse incoming requests
    // - Execute requested operations
    // - Generate appropriate responses
    
    try {
        std::cout << "Handling service request: " << request << std::endl;
        
        if (request == "get_state") {
            response = createStateMessage(current_state_);
        }
        else if (request == "get_mode") {
            response = createModeMessage(current_mode_);
        }
        else if (request.find("set_mode:") == 0) {
            std::string mode_str = request.substr(9);
            Mode new_mode = parseModeFromString(mode_str);
            setMode(new_mode);
            response = "OK";
        }
        else {
            response = "UNKNOWN_REQUEST";
        }
        
    } catch (const std::exception& e) {
        response = "ERROR: " + std::string(e.what());
    }
}


