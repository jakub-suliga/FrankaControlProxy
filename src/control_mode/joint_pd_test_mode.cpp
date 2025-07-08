#include "joint_pd_test_mode.hpp"
#include <franka/exception.h>
#include <iostream>

JointPDControlMode::JointPDControlMode() = default;
JointPDControlMode::~JointPDControlMode() = default;

void JointPDControlMode::start() {
    std::cout << "[JointPDControlMode] Started.\n";
    is_running_ = true;

    if (!robot_ || !model_) {
        std::cerr << "[JointPDControlMode] Robot or model not set.\n";
        return;
    }

    const std::array<double, 7> Kp = {60.0, 60.0, 60.0, 50.0, 20.0, 10.0, 10.0};
    const std::array<double, 7> Kd = {5.0, 5.0, 5.0, 3.0, 1.0, 0.5, 0.5};

    std::function<franka::Torques(const franka::RobotState&, franka::Duration)> torque_callback =
        [this, Kp, Kd](const franka::RobotState& state, franka::Duration) -> franka::Torques {
            if (!is_running_) {
                throw franka::ControlException("JointPDControlMode stopped.");
            }

            setCurrentState(state);

            auto leader_ptr = getLeaderState();
            if (!leader_ptr) {
                std::cerr << "[JointPDControlMode] No leader state available.\n";
                return franka::Torques({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            }

            const franka::RobotState& leader = *leader_ptr;
            std::array<double, 7> tau{};
            for (size_t i = 0; i < 7; ++i) {
                double pos_error = leader.q[i] - state.q[i];
                double vel_error = leader.dq[i] - state.dq[i];
                tau[i] = Kp[i] * pos_error + Kd[i] * vel_error;
            }

            return franka::Torques(tau);
        };

    try {
        robot_->control(torque_callback);
    } catch (const franka::Exception& e) {
        std::cerr << "[JointPDControlMode] Exception: " << e.what() << std::endl;
    }

    std::cout << "[JointPDControlMode] Exited.\n";
}

void JointPDControlMode::stop() {
    is_running_ = false;
}
