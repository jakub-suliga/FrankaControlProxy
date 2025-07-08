#include "FrankaProxy.hpp"
#include "protocol/codec.hpp"
#include "protocol/franka_arm_state.hpp"
#include "protocol/franka_gripper_state.hpp"
#include "protocol/msg_id.hpp"
#include "protocol/message_header.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <algorithm>
#include "RobotConfig.hpp"
#include "debugger/state_debug.hpp"
#include"protocol/mode_id.hpp"
static std::atomic<bool> running_flag{true};  // let ctrl-c stop the server
static void signalHandler(int signum) {
    std::cout << "\n[INFO] Caught signal " << signum << ", shutting down..." << std::endl;
    running_flag = false;
}
franka::RobotState default_state;
default_state.q = {{0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785}};  // 默认初始位置
default_state.O_T_EE = {{
    1.0, 0.0, 0.0, 0.3,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.5,
    0.0, 0.0, 0.0, 1.0
}};


////main Proxy
FrankaProxy::FrankaProxy(const std::string& config_path)
    : context_(1),// Initialize ZMQ context with 2 I/O threads,it can be adjusted
    pub_socket_(context_, ZMQ_PUB), 
    sub_socket_(context_, ZMQ_SUB), 
    rep_socket_(context_, ZMQ_REP), 
    in_control_(false) {
    initialize(config_path);
}

void FrankaProxy::initialize(const std::string& filename) {
    RobotConfig config(filename);
    type_ = config.getValue("type", "Arm"); // Default to "Arm" if not specified
    robot_ip_ = config.getValue("robot_ip");
    follower_ = config.getValue("follower", false); // Default to false if not specified
    rep_socket_.set(zmq::sockopt::rcvtimeo, SOCKET_TIMEOUT_MS);
    sub_socket_.set(zmq::sockopt::rcvtimeo, SOCKET_TIMEOUT_MS);
    if (type_ == "Arm")
    {
        state_pub_addr_ = config.getValue("state_pub_addr");
        pub_socket_.bind(state_pub_addr_);
        service_addr_ = config.getValue("service_addr");
        rep_socket_.bind(service_addr_);
        robot_ = std::make_shared<franka::Robot>(robot_ip_);
        model_ = std::make_shared<franka::Model>(robot_->loadModel());
        if(follower_){
            state_sub_addr_ = config.getValue("state_sub_addr");
            std::cout << "[FrankaProxy] Initializing Arm Follower Proxy with IP: " << robot_ip_ << std::endl;
            sub_socket_.connect(state_sub_addr_);
            state_sub_thread_ = std::thread(&FrankaProxy::stateSubscribeThread, this);
            leader_state_ = default_state;
        } else {
            std::cout << "[FrankaProxy] Initializing Arm Leader Proxy with IP: " << robot_ip_ << std::endl;
        }
    }
    else if (type_ == "Gripper")
    {
        gripper_pub_addr_ = config.getValue("gripper_pub_addr");
        pub_socket_.bind(gripper_pub_addr_);
        service_addr_ = config.getValue("service_addr");
        rep_socket_.bind(service_addr_);
        gripper_ = std::make_shared<franka::Gripper>(robot_ip_);
        if(follower_){
            gripper_sub_addr_ = config.getValue("gripper_sub_addr");
            std::cout << "[FrankaProxy] Initializing Gripper Follower Proxy with IP: " << robot_ip_ << std::endl;
            sub_socket_.connect(gripper_sub_addr_);
            gripper_sub_thread_ = std::thread(&FrankaProxy::gripperSubscribeThread, this);
        }
        else
        {
            std::cout << "[FrankaProxy] Initializing Gripper Leader Proxy with IP: " << robot_ip_ << std::endl;
        }
        std::cout << "[FrankaProxy] Initializing Gripper Proxy with IP: " << robot_ip_ << std::endl;
    }
    else
    {
        throw std::runtime_error("Unsupported type in config: " + type_);
    }

}

FrankaProxy::~FrankaProxy() {
    stop();
}

////basic function
//start and initialize threads
bool FrankaProxy::start() {
    in_control_ = True;
    if (type_ == "Arm") {
        if(follower_) {
            state_sub_thread_ = std::thread(&FrankaProxy::stateSubscribeThread, this);
            std::cout << "done arm sub"<< std::endl;
        } 
        return startArm();
        
    } else if (type_ == "Gripper") {
        if(follower_) {
            gripper_sub_thread_ = std::thread(&FrankaProxy::gripperSubscribeThread, this);
            std::cout << "done gripper sub"<< std::endl;
        }
        return startGripper();
    } else {
        std::cerr << "[ERROR] Unsupported type: " << type_ << std::endl;
        return false;
    }

}

bool FrankaProxy::startArm(){
    in_control_ = true;
    std::cout << in_control_ <<"control"<< std::endl;
    std::cout << robot_<<"robot"<< std::endl;
    current_state_ = robot_->readOnce();
    state_pub_thread_ = std::thread(&FrankaProxy::statePublishThread, this);
    std::cout << "done arm pub"<< std::endl;
    control_thread_ = std::thread(&FrankaProxy::controlLoopThread, this);
    std::cout << "done control loop"<< std::endl;
    {
    std::lock_guard<std::mutex> lock(control_mutex_);
    current_mode_ = control_modes_map_["idle"].get();
    }

    service_thread_ = std::thread(&FrankaProxy::responseSocketThread, this);
    std::cout << "done service"<< std::endl;
    return true;
}

bool FrankaProxy::startGripper() {
    gripper_pub_thread_ = std::thread(&FrankaProxy::gripperPublishThread, this);
    std::cout << "done gripper pub"<< std::endl;
    service_thread_ = std::thread(&FrankaProxy::responseSocketThread, this);
    std::cout << "done service"<< std::endl;
}

//stop sever and clean up resource
void FrankaProxy::stop() {
    std::cout << "[INFO] Stopping FrankaProxy...\n";
    if (type_ == "Arm") {
        return stopArm();
        std::cout << "[INFO] Shutdown complete.\n"
    } else if (type_ == "Gripper") {
        return stopGripper();
        std::cout << "[INFO] Shutdown complete.\n"
    } else {
        std::cerr << "[ERROR] Unsupported type: " << type_ << std::endl;
        return false;
    }
}

void FrankaProxy::stopArm() {
    in_control_ = false;
    current_mode_->stop();
    // try close ZeroMQ sockets
    try {
        pub_socket_.close();
        rep_socket_.close();
        sub_socket_.close();
    } catch (const zmq::error_t& e) {
        std::cerr << "[ZMQ ERROR] " << e.what() << "\n";
    }
    // wait for closing
    if (state_pub_thread_.joinable()) state_pub_thread_.join();
    if (service_thread_.joinable()) service_thread_.join();
    if (control_thread_.joinable()) control_thread_.join();
    if (state_sub_thread_.joinable()) state_sub_thread_.join();//if not a follower, this thread will not be created, but no hurt
    robot_.reset();  // release robot and model
    model_.reset();
    current_mode_ = nullptr;// reset current control mode
    
}

void FrankaProxy::stopGripper() {
    in_control_ = false;
    // try close ZeroMQ sockets
    try {
        pub_socket_.close();
        rep_socket_.close();
        sub_socket_.close();
    } catch (const zmq::error_t& e) {
        std::cerr << "[ZMQ ERROR] " << e.what() << "\n";
    }
    // wait for closing
    if (gripper_pub_thread_.joinable()) gripper_pub_thread_.join();
    if (service_thread_.joinable()) service_thread_.join();
    if (gripper_sub_thread_.joinable()) gripper_sub_thread_.join();//if not a follower, this thread will not be created, but no hurt
    gripper_.reset();  // release gripper
}


// Main loop for processing requests, ctrl-c to stop the server
void FrankaProxy::spin() {
    std::signal(SIGINT, signalHandler);  //  Catch Ctrl+C to stop the server
    std::cout << "[INFO] Entering main spin loop. Press Ctrl+C to exit." << std::endl;
    std::cout << "running_flag" <<running_flag << std::endl;
    while (running_flag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    stop(); 
    std::cout << "[INFO] Shutdown complete.\n";
}

////get current arm & gripper state
//get current arm state
franka::RobotState FrankaProxy::getCurrentState() {
    std::lock_guard<std::mutex> lock(control_mutex_);
    if (current_mode_) {
        return current_mode_->getRobotState();//robot should always in a control mode, when there is no command it is idle
    } else {
        throw std::runtime_error("No control mode active.");
    }
}
//get current gripper state
protocol::FrankaGripperState FrankaProxy::getCurrentGripper(){
    franka::Gripper gripper(gripper_);
    franka::GripperState gripper_state = gripper.readOnce();
    protocol::FrankaGripperState proto_gripper_state = protocol::FrankaGripperState::fromGripperState(gripper_state);
    return proto_gripper_state;
}


//// Control mode register
void FrankaProxy::registerControlMode(const std::string& mode, std::unique_ptr<AbstractControlMode> control_mode) {
    control_modes_map_[mode] = std::move(control_mode);
    std::cout << "Registered mode: " << mode << "\n";
}


//// with RobotConfig
void FrankaProxy::displayConfig() const {
    std::cout << proxy_config_ << std::endl;
}

//// publish threads
void FrankaProxy::statePublishThread() {
    while (in_control_) {
        protocol::FrankaArmState proto_state = protocol::FrankaArmState::fromRobotState(getCurrentState());
        //auto t_start = std :: chrono::high_resolution_clock::now();
        auto msg = protocol::encodeStateMessage(proto_state);
        //debug:check with the message
        //protocol::debugPrintFrankaArmStateBuffer(msg);
        pub_arm_socket_.send(zmq::buffer(msg), zmq::send_flags::none);
        //auto t_end = std::chrono::high_resolution_clock::now();
        //auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
        //check the speed of decode and pack the message
        //std::cout << "[FrankaProxy] encodeStateMessage took " << duration_us << " us" << std::endl;
        //std::this_thread::sleep_for(std::chrono::milliseconds(1000 / STATE_PUB_RATE_HZ));
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

void FrankaProxy::gripperPublishThread() {
    while (in_control_) {
        protocol::FrankaGripperState proto_gripper_state = getCurrentGripper();
        auto t_start = std::chrono::high_resolution_clock::now();
        auto msg = protocol::encodeGripperMessage(proto_gripper_state);
        //debug:check with the message
        //protocol::debugPrintFrankaGripperStateBuffer(msg);//maybe need
        pub_socket_.send(zmq::buffer(msg), zmq::send_flags::none);
        auto t_end = std::chrono::high_resolution_clock::now();
        auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
        //check the speed of decode and pack the message
        std::cout << "[FrankaProxy] encodeGripperMessage took " << duration_us << " us" << std::endl;
        //std::this_thread::sleep_for(std::chrono::milliseconds(1000 / GRIPPER_PUB_RATE_HZ));
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

/// subscribe threads(just for follower)
void FrankaProxy::stateSubscribeThread() {
    sub_socket_.set(zmq::sockopt::subscribe, ""); // Subscribe to all messages
    while (in_control_) {
        try{
            zmq::message_t message;
            sub_socket_.recv(message, zmq::recv_flags::none);
            std::vector<uint8_t> data(static_cast<uint8_t*>(message.data()), static_cast<uint8_t*>(message.data()) + message.size());
            // Debug: Print the received message size
            std::cout << "[FrankaProxy] Received state update: size = " << data.size() << std::endl;
            protocol::FrankaArmState arm_state;
            if(bool decodeStateMessage(data, arm_state))
            {
                leader_state_ = arm_state.toRobotState(); // Convert to RobotState for follower
                {
                    std::lock_guard<std::mutex> lock(control_mutex_);
                    if (current_mode_) {
                    current_mode_->setLeaderState(leader_state_);
                }
                std::cout << "[FrankaProxy] Received state update from leader." << std::endl;
                }
            }
            else{
                std::cerr << "[FrankaProxy] Failed to decode state message." << std::endl;
            }
        } catch (const zmq::error_t& e) {
            std::cerr << "[FrankaProxy] ZMQ recv error: " << e.what() << std::endl;
            break;
        }
    }
}


void FrankaProxy::gripperSubscribeThread() {
    sub_socket_.set(zmq::sockopt::subscribe, ""); // Subscribe to all messages 
    try{
        zmq::message_t message;
        sub_socket_.recv(message, zmq::recv_flags::none);
        std::vector<uint8_t> data(static_cast<uint8_t*>(message.data()), static_cast<uint8_t*>(message.data()) + message.size());
        // Debug: Print the received message size
        std::cout << "[FrankaProxy] Received gripper update: size = " << data.size() << std::endl;
        protocol::FrankaGripperState gripper_state;
        if (protocol::decodeGripperMessage(data, gripper_state)) {
            leader_gripper_state_ = gripper_state.toGripperState(); // Convert to GripperState for follower
            std::cout << "[FrankaProxy] Received gripper update from leader." << std::endl;
        } else {
            std::cerr << "[FrankaProxy] Failed to decode gripper message." << std::endl;
        }
    }catch (const zmq::error_t& e) {
        std::cerr << "[FrankaProxy] ZMQ recv error: " << e.what() << std::endl;
        break;
    }
}

//// control loop thread (just for arm)
void FrankaProxy::controlLoopThread() {
    std::cout << "[ControlLoop] Thread started.\n";

    while (in_control_) {
        //share current_mode_ with service thread
        std::cout << "in_control"<<in_control_ << std::endl;
        AbstractControlMode* mode = nullptr;
        {
            std::lock_guard<std::mutex> lock(control_mutex_);
            mode = current_mode_;
        }//control_thread read current_mode_
        //std::cout << current_mode_ << "current_mode"<<std::endl;
        //protection from nullptr

        if (!mode) {
            std::cerr << "[Warning] No control mode selected. Waiting...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        try {
            mode->setRobot(robot_);
            mode->setModel(model_);
            mode->start();  //block with franka::Robot::control() until stop() is called by service thread
        } catch (const std::exception& e) {
            std::cerr << "[Exception] Control mode failed, return idle: " << e.what() << "\n";
                // in_control_ = false;
                std::lock_guard<std::mutex> lock(control_mutex_);
                current_mode_ = control_modes_map_["idle"].get();
            }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "[ControlLoop] Thread exited.\n";
}



void FrankaProxy::setControlMode(const std::string& mode) {
    auto it = control_modes_map_.find(mode);
    if (it == control_modes_map_.end()) {
        std::cerr << "[Error] Control mode not found: " << mode << std::endl;
        return;
    }

    {
        std::lock_guard<std::mutex> lock(control_mutex_);

        if (current_mode_) {
            std::cout << "[Info] Stopping previous control mode...\n";
            current_mode_->stop();  // stopMotion + is_running_ = false
        }

        current_mode_ = it->second.get();

        std::cout << "[Info] Switched to control mode: " << mode << std::endl;
    }
}

////response socket thread, used for service request
void FrankaProxy::responseSocketThread() {
    while (in_control_) {
        //get request from client
        zmq::message_t request;
        if (!rep_socket_.recv(request, zmq::recv_flags::none)) continue;//skip,if fail
        
        std::vector<uint8_t> req_data(static_cast<uint8_t*>(request.data()),//begin
                                      static_cast<uint8_t*>(request.data()) + request.size())//end;
        std::cout << "[FrankaProxy] Received request: msg size = " << req_data.size() << std::endl;

        //std::string response;
        std::vector<uint8_t> response;
        handleServiceRequest(req_data, response);
        //send response
        rep_socket_.send(zmq::buffer(response), zmq::send_flags::none);
    }
}


//todo:error message have not done
void FrankaProxy::handleServiceRequest(const std::vector<uint8_t>& request, std::vector<uint8_t>& response) {
    //check the request size 
    using namespace protocol;
    if (request.size() < 4) {
        auto response = encodeErrorMessage(0x01);
        return;
    }

    //parse the header
    const uint8_t* data = reinterpret_cast<const uint8_t*>(request.data());
    MessageHeader header = MessageHeader::decode(data);
    if (request.size() != 4 + header.len) {
        auto err = encodeErrorMessage(0x02);
        //response.assign(reinterpret_cast<const char*>(err.data()), err.size());
        response = err;
        return;
    }
    
    const uint8_t* payload = data + 4;
    std::vector<uint8_t> resp;
    //deal with different kinds of msg
    switch (header.id) {
        case static_cast<int> (protocol::MsgID::GET_STATE_REQ):
            {

                resp = encodeStateMessage(protocol::FrankaArmState::fromRobotState(getCurrentState()));//send protocol_state to pack message
                break;
            }
        case static_cast<int> (protocol::MsgID::GET_CONTROL_MODE_REQ):

            resp = encodeModeMessage(5); //mode in string
            break;

        case static_cast<int> (protocol::MsgID::SET_CONTROL_MODE_REQ):
            {
                std::cout<<"header in control"<<header.len<<std::endl;
                if (header.len < 1) {
                    std::cout<<"header in control"<<header.len<<std::endl;
                    resp = encodeErrorMessage(0x03);
                    break;
                }

                protocol::ModeID mode_id = static_cast<ModeID>(payload[0]);
                std::string mode_str = protocol::toString(mode_id);
                std::cout<<" received the control_req";
                setControlMode(mode_str);
                bool success = true;
                resp = encodeStartControlResp(success, mode_id);
                break;
            }
        case static_cast<int> (protocol::MsgID::GET_SUB_PORT_REQ):
            //resp = encodeMessage(MessageHeader{protocol::MsgID::GET_SUB_PORT_RESP, 2}, {0x0C, 0x34}); // TODO: dynamic port
            break;
        case static_cast<int> (protocol::MsgID::GRIPPER_COMMAND_REQ):
            {
                // Handle gripper command request
                if (header.len < 1) {
                    resp = encodeErrorMessage(0x04);
                    break;
                }
                // todo:implement gripper command handling with function
                //just the test:
                uint8_t command = payload[0];
                if (command == 0) { // Example command, replace with actual logic
                    gripper_->stop(); // Stop the gripper
                    std::cout << "Gripper stop command received." << std::endl;
                } else if (command == 1) {
                    gripper_->move(0.08, 0.05); // Move gripper to 8cm with speed 5cm/s
                    std::cout << "Gripper move command received." << std::endl;
                } else {
                    resp = encodeErrorMessage(0x05); // Invalid command
                }
        default:
            resp = encodeErrorMessage(0xFF);
            break;
    }

    response.assign(reinterpret_cast<const char*>(resp.data()), resp.size());
}
}