#ifndef CONTROL_MODE_H
#define CONTROL_MODE_H

#include <franka/robot.h>
#include <franka/model.h>

#include "FrankaState.h"

class AbstractControlMode {
public:
    // Virtual destructor for proper cleanup in derived classes
    virtual ~AbstractControlMode() = default;

    // Pure virtual public functions
    virtual void start() = 0;
    virtual void stop() = 0;

    void setRobot(std::shared_ptr<franka::Robot> robot) {
        robot_ = std::move(robot);
    }

    void setModel(std::shared_ptr<franka::Model> model) {
        model_ = std::move(model);
    }

protected:
    // Protected constructor to prevent direct instantiation
    AbstractControlMode() = default;

    // Protected setup function for derived classes
    std::shared_ptr<franka::Robot> robot_;
    std::shared_ptr<franka::Model> model_;
    std::shared_ptr<FrankaArmState> current_state_;
    bool is_running_ = false;

};

#endif // CONTROL_MODE_H