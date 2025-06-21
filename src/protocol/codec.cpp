#include "protocol/codec.hpp"
#include "protocol/franka_arm_state.hpp"
namespace protocol {

// header + payload
std::vector<uint8_t> encodeMessage(const MessageHeader& header, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> result(4 + payload.size());
    header.encode(result.data());  // get head
    std::memcpy(result.data() + 4, payload.data(), payload.size());
    return result;
}

// GET_STATE_RESP
std::vector<uint8_t> encodeStateMessage(const protocol::FrankaArmState& state) {
    auto payload = state.encode();  // 648B
    MessageHeader header{
        static_cast<uint8_t>(MsgID::GET_STATE_RESP), 
        static_cast<uint16_t>(payload.size())
    };
    return encodeMessage(header, payload);
}

// QUERY_STATE_RESP
std::vector<uint8_t> encodeModeMessage(uint8_t mode_code) {
    std::vector<uint8_t> payload{mode_code}; 
    MessageHeader header{
        static_cast<uint8_t>(MsgID::QUERY_STATE_RESP),
        static_cast<uint16_t>(payload.size())
    }; // 1 Byte for mode code
    return encodeMessage(header, payload);
}


//START_CONTROL_RESP
// std::vector<uint8_t> encodeRespcontrolMessage() {

// }


// ERROR
std::vector<uint8_t> encodeErrorMessage(uint8_t error_code) {
    std::vector<uint8_t> payload{error_code};
    MessageHeader header{
        static_cast<uint8_t>(MsgID::ERROR),
        static_cast<uint16_t>(payload.size())
    };
    return encodeMessage(header, payload);
}

}  // namespace protocol