#ifndef ABSTRACT_CONTROL_MODE_H
#define ABSTRACT_CONTROL_MODE_H

#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <franka/exception.h>
class AbstractControlMode {
public:
    // Virtual destructor for proper cleanup in derived classes
    virtual ~AbstractControlMode() = default;
    // Pure virtual public functions
    //virtual void initialize(const RobotState& initial_state);
    virtual void start() = 0;
    virtual void stop() = 0;

    void setRobot(std::shared_ptr<franka::Robot> robot) {
        robot_ = std::move(robot);
    }
    void setModel(std::shared_ptr<franka::Model> model) {
        model_ = std::move(model);
    }
    virtual franka::RobotState UpdateState() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return current_state_;
    }
protected:
    // Protected constructor to prevent direct instantiation
    AbstractControlMode() = default;

    // Protected setup function for derived classes
    std::shared_ptr<franka::Robot> robot_;
    std::shared_ptr<franka::Model> model_;
    franka::RobotState current_state_;
    mutable std::mutex state_mutex_;
    bool is_running_ = false;

};

#endif // ABSTRACT_CONTROL_MODE_H