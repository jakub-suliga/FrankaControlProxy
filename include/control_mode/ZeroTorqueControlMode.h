#include <string>

#include "AbstractControlMode.h"

class ZeroTorqueControlMode : public AbstractControlMode {
public:
    // Constructor
    explicit ZeroTorqueControlMode() {
    }

    // Destructor
    ~ZeroTorqueControlMode() override = default;

    // Start the control mode
    void start() override {
        // Set up any necessary parameters for zero torque control
        robot_->setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 300.0, 300.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 300.0, 300.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
        is_running_ = true;
        std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
            joint_torque_callback = [this](const franka::RobotState &state, franka::Duration) -> franka::Torques
        {
            // TODO: quit the mode
            if (!is_running_) {
                return ;
            }
            franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
            this->current_state_->updateState(state);
            zero_torques = this->model_->coriolis(state);        
            return zero_torques;
        };
        robot_->control(joint_torque_callback);
    }

    // Stop the control mode
    void stop() override {
        // Cleanup if necessary
    }

protected:
    // Setup control mode specifics

};
