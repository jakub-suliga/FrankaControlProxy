#include "FrankaProxy.hpp"
#include "protocol/codec.hpp"
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

FrankaProxy::FrankaProxy(const std::string& config_path)
    : context_(1), 
    pub_socket_(context_, ZMQ_PUB), 
    sub_socket_(context_, ZMQ_SUB), 
    rep_socket_(context_, ZMQ_REP), 
    in_control_(false) {
    initialize(config_path);
    setupCommunication();
}

FrankaProxy::~FrankaProxy() {
    stop();
}

//start and initialize threads
bool FrankaProxy::start() {
    in_control_ = true;
    std::cout << in_control_ <<"control"<< std::endl;
    std::cout << robot_<<"robot"<< std::endl;
    current_state_ = robot_->readOnce();
    //std::cout << current_state_<< std::endl;
    control_thread_ = std::thread(&FrankaProxy::controlLoopThread, this);
    std::cout << "done control"<< std::endl;
    state_pub_thread_ = std::thread(&FrankaProxy::statePublishThread, this);
    std::cout << "done pub"<< std::endl;
    service_thread_ = std::thread(&FrankaProxy::responseSocketThread, this);
    std::cout << "done rep"<< std::endl;
    return true;
}

void FrankaProxy::stop() {
    std::cout << "[INFO] Stopping FrankaProxy...\n";
    in_control_ = false;
    // try close ZeroMQ sockets
    try {
        pub_socket_.close();
        rep_socket_.close();
        //sub_socket_.close();
    } catch (const zmq::error_t& e) {
        std::cerr << "[ZMQ ERROR] " << e.what() << "\n";
    }
    // wait for closing
    if (state_pub_thread_.joinable()) state_pub_thread_.join();
    if (service_thread_.joinable()) service_thread_.join();
    if (control_thread_.joinable()) control_thread_.join();
    robot_.reset();  // release robot and model
    model_.reset();
    //std::cout << "[INFO] Shutdown complete.\n";
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

franka::RobotState FrankaProxy::getCurrentState() {
    std::lock_guard<std::mutex> lock(control_mutex_);
    if (current_mode_) {
        return current_mode_->UpdateState();
    } else {
        throw std::runtime_error("No control mode active.");
    }
}
// Control mode management
void FrankaProxy::registerControlMode(const std::string& mode, std::unique_ptr<AbstractControlMode> control_mode) {
    control_modes_map_[mode] = std::move(control_mode);
    std::cout << "Registered mode: " << mode << "\n";
}

void FrankaProxy::setControlMode(const std::string& mode) {
    auto it = control_modes_map_.find(mode);
    if (it != control_modes_map_.end()) {
        {
            std::lock_guard<std::mutex> lock(control_mutex_);
            current_mode_ = it->second.get();
        }
        mode_name = mode;
        std::cout << "Switched to control mode: " << mode << std::endl;
    } else {
        std::cerr << "Control mode not found: " << mode << std::endl;
    }
}



void FrankaProxy::displayConfig() const {
    std::cout << proxy_config_ << std::endl;
}

// with RobotConfig
void FrankaProxy::initialize(const std::string& filename) {
    RobotConfig config(filename);
    robot_ip_ = config.getValue("robot_ip");
    state_pub_addr_ = config.getValue("state_pub_addr");
    service_addr_ = config.getValue("service_addr");
    robot_ = std::make_shared<franka::Robot>(robot_ip_);
    model_ =std::make_shared<franka::Model>(robot_->loadModel());
}

void FrankaProxy::setupCommunication() {
    const int timeout_ms = 100;
    rep_socket_.set(zmq::sockopt::rcvtimeo,timeout_ms);
    
    pub_socket_.bind(state_pub_addr_);
    rep_socket_.bind(service_addr_);

    // sub_socket_ is unused for now
}


void FrankaProxy::statePublishThread() {
    while (in_control_) {
        protocol::FrankaArmState proto_state = protocol::FrankaArmState::fromRobotState(getCurrentState());
        auto t_start = std :: chrono::high_resolution_clock::now();
        auto msg = protocol::encodeStateMessage(proto_state);
        //debug:check with the message
        //protocol::debugPrintFrankaArmStateBuffer(msg);
        pub_socket_.send(zmq::buffer(msg), zmq::send_flags::none);
        auto t_end = std::chrono::high_resolution_clock::now();
        auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
        //check the speed of decode and pack the message
        //std::cout << "[FrankaProxy] encodeStateMessage took " << duration_us << " us" << std::endl;
        //std::this_thread::sleep_for(std::chrono::milliseconds(1000 / STATE_PUB_RATE_HZ));
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

void FrankaProxy::responseSocketThread() {
    while (in_control_) {
        zmq::message_t request;
        if (!rep_socket_.recv(request, zmq::recv_flags::none)) continue;//skip,if fail
        std::string req_str(static_cast<char*>(request.data()), request.size());
        std::string response;

        handleServiceRequest(req_str, response);
        rep_socket_.send(zmq::buffer(response), zmq::send_flags::none);
    }
}
void FrankaProxy::controlLoopThread() {
    AbstractControlMode* last_mode = nullptr;
    std::cout << "in_control"<<in_control_ << std::endl;
    while (in_control_) {
        AbstractControlMode* mode = nullptr;
        {
            std::lock_guard<std::mutex> lock(control_mutex_);
            mode = current_mode_;
        }//control_thread read current_mode_
        //std::cout << current_mode_ << "current_mode"<<std::endl;
        mode->setRobot(robot_);
        mode->setModel(model_);
        if (!mode) {
            std::cerr << "[Warning] No control mode selected. Waiting...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        if (mode!= last_mode) {
            std::cout << "[Info] Switching control mode...\n";
            try {
                if (last_mode) {
                    last_mode->stop();
                }
                last_mode = mode;
                mode->start();         // 
            } 
            catch (const std::exception& e) {
                std::cerr << "[Exception] Control mode failed: " << e.what() << "\n";
                in_control_ = false;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (last_mode) {
        last_mode->stop();
    }

    std::cout << "[Info] Control loop thread exited.\n";
}


//todo:error message have not done
void FrankaProxy::handleServiceRequest(const std::string& request, std::string& response) {
    using namespace protocol;
    if (request.size() < 4) {
        auto err = encodeErrorMessage(0x01);
        response.assign(reinterpret_cast<const char*>(err.data()), err.size());
        return;
    }
    const uint8_t* data = reinterpret_cast<const uint8_t*>(request.data());
    MessageHeader header = MessageHeader::decode(data);
    if (request.size() != 4 + header.len) {
        auto err = encodeErrorMessage(0x02);
        response.assign(reinterpret_cast<const char*>(err.data()), err.size());
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
        case static_cast<int> (protocol::MsgID::QUERY_STATE_REQ):

            resp = encodeModeMessage(5); //now modename is string, need to transfer after building the control_mode_id
            break;

        case static_cast<int> (protocol::MsgID::START_CONTROL_REQ):
            {
                std::cout<<"header in control"<<header.len<<std::endl;
                if (header.len < 1) {
                    std::cout<<"header in control"<<header.len<<std::endl;
                    resp = encodeErrorMessage(0x03);
                    break;
                }

                // TODO: resp the control_req
                protocol::ModeID mode_id = static_cast<ModeID>(payload[0]);
                std::string mode_str = protocol::toString(mode_id);
                std::cout<<" received the control_req";
                setControlMode(mode_str);
                break;
            }
        case static_cast<int> (protocol::MsgID::GET_SUB_PORT_REQ):
            //resp = encodeMessage(MessageHeader{protocol::MsgID::GET_SUB_PORT_RESP, 2}, {0x0C, 0x34}); // TODO: dynamic port
            break;

        default:
            resp = encodeErrorMessage(0xFF);
            break;
    }

    response.assign(reinterpret_cast<const char*>(resp.data()), resp.size());
}
