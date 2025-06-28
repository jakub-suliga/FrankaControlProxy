#ifndef IDLE_CONTROL_MODE_H
#define IDLE_CONTROL_MODE_H

#include "abstract_control_mode.h"
#include <memory>
#include <iostream>

class IdleControlMode : public AbstractControlMode {
public:
    IdleControlMode() = default;
    ~IdleControlMode() override = default;

    //void initialize(const franka::RobotState& initial_state) override;
    void start() override;
    void stop() override;

private:
    void updateOnce();  // 使用 readOnce 读取当前状态
};

#endif // IDLE_CONTROL_MODE_H