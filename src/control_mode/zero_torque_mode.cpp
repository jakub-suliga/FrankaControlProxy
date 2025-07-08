#include "zero_torque_mode.hpp"
void ZeroTorqueMode::start() {
        is_running_ = true;
        std::cout << "[ZeroTorqueMode] Started.\n";
        if (!robot_ || !model_) {
            std::cerr << "[ZeroTorqueMode] Robot or model not set.\n";
            return;
        }
        robot_->setCollisionBehavior(
            {{100.0, 100.0, 100.0, 100.0, 100.0, 300.0, 300.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 300.0, 300.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
        std::function<franka::Torques(const franka::RobotState&, franka::Duration)> torque_callback =
            [this](const franka::RobotState& state, franka::Duration) -> franka::Torques {
                if (!is_running_) {
                    throw franka::ControlException("ZeroTorqueControlMode stopped.");
                }

                setCurrentState(state);


            // test get leader state
            auto leader_ptr = getLeaderState();
            if (leader_ptr) {
            const auto& leader = *leader_ptr;

            // print leader state
            std::cout << "[Leader q] ";
            for (double q_i : leader.q) {
                std::cout << q_i << " ";
            }
            std::cout << std::endl;
        } else {
            std::cout << "[ZeroTorqueMode] No leader state available." << std::endl;
        }
                std::array<double, 7> zero_torque = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                return franka::Torques(zero_torque);
            };

        //start control callback
        try {
            robot_->control(torque_callback);
        } catch (const franka::Exception& e) {
            std::cerr << "[ZeroTorqueMode] Exception: " << e.what() << std::endl;
        }

        std::cout << "[ZeroTorqueMode] Exited.\n";
    }

    void ZeroTorqueMode::stop() {
        is_running_ = false;
        std::cout << "[ZeroTorqueMode] Stopping Zero Torque mode." << std::endl;
    }

