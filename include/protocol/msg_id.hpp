#ifndef MSG_ID_HPP
#define MSG_ID_HPP
#include <cstdint>

namespace protocol {

enum class MsgID : uint8_t {
    // Client → Server
    GET_STATE_REQ      = 0x01,  // Request a single FrankaArmState
    QUERY_STATE_REQ    = 0x02,  // Ask for active control mode
    START_CONTROL_REQ  = 0x03,  // Switch to desired mode
    GET_SUB_PORT_REQ   = 0x04,  // Query PUB port number

    // Server → Client
    GET_STATE_RESP     = 0x51,  //Respond to GET_STATE_REQ with FrankaArmState
    QUERY_STATE_RESP   = 0x52,  //Respond to QUERY_STATE_REQ (1 byte: ControlMode)
    START_CONTROL_RESP = 0x53,  //Respond to START_CONTROL_REQ (1 byte: status,0 = OK)
    GET_SUB_PORT_RESP  = 0x54,  // Respond to GET_SUB_PORT_REQ (2 bytes: port number)

    // Server → Client (error)
    ERROR              = 0xFF   // 1 byte error code
    //error details
};

}  // namespace protocol
#endif // MSG_ID_HPP