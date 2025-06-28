#include "idle_control_mode.hpp"

// void IdleControlMode::initialize(const franka::RobotState& initial_state) {
//     //todo:write recover
    
//     current_state_ = initial_state;
//     std::cout << "[IdleControlMode] Initialized with initial state." << std::endl;
// }
void IdleControlMode::start() {
    if (!robot_) {
        throw std::runtime_error("[IdleControlMode] Robot not set.");
    }
    std::cout << "[IdleControlMode] Entering idle mode. Reading state once..." << std::endl;
    is_running_ = true;
    // while(is_running_)
    // {
    //     updateOnce();
    // }
    updateOnce();
    std::cout << "[IdleControlMode] Idle mode active. No control actions taken." << std::endl;
}
void IdleControlMode::stop() {
    is_running_ = false;
    std::cout << "[IdleControlMode] Stopped idle mode." << std::endl;
}
void IdleControlMode::updateOnce() {
    try {
        franka::RobotState new_state = robot_->readOnce();
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state_ = new_state;
        }
        //std::cout << current_state_<< std::endl;
    } catch (const franka::Exception& e) {
        std::cerr << "[IdleControlMode] Failed to read robot state: " << e.what() << std::endl;
    }
}
