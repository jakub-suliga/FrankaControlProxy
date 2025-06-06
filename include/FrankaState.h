#ifndef FRANKA_ARM_STATE_HPP
#define FRANKA_ARM_STATE_HPP

#include <array>
#include <string>
#include <chrono>
#include <mutex>
#include <franka/robot_state.h>

struct FrankaArmState {
    // Joint space data (7 DOF)
    std::array<double, 7> joint_positions;      // q - Joint positions [rad]
    std::array<double, 7> joint_velocities;     // dq - Joint velocities [rad/s]
    std::array<double, 7> joint_torques;        // tau_J - Joint torques [Nm]
    std::array<double, 7> external_torques;     // tau_ext_hat_filtered - External torques [Nm]
    
    // Cartesian space data
    std::array<double, 16> cartesian_pose;      // O_T_EE - End-effector pose (4x4 matrix flattened)
    std::array<double, 6> cartesian_velocity;   // O_dP_EE - End-effector velocity [m/s, rad/s]
    
    // Metadata
    double timestamp;                           // Timestamp in seconds
    bool is_connected;                          // Connection status
    
    // Thread safety
    mutable std::mutex mutex;
    
    // Default constructor
    FrankaArmState() 
        : joint_positions{}
        , joint_velocities{}
        , joint_torques{}
        , external_torques{}
        , cartesian_pose{}
        , cartesian_velocity{}
        , timestamp(0.0)
        , is_connected(false) {
    }
    
    // TODO: Update state from franka::RobotState
    void updateState(const franka::RobotState& robot_state);

    // Serialize to JSON string
    char encode();
    
    // Static function to deserialize from JSON string
    static FrankaArmState deserialize(const char json_str);
};

// TODO: implement FrankaGripperState
struct FrankaGripperState
{
    /* data */
};



#endif // FRANKA_ARM_STATE_HPP