#include "protocol/message_header.hpp"

namespace protocol {

void MessageHeader::encode(uint8_t* buffer) const {
    buffer[0] = id;
    buffer[1] = (len >> 8) & 0xFF;  // high byte
    buffer[2] = len & 0xFF;         // low byte
    buffer[3] = 0;                  // pad is always 0
}

MessageHeader MessageHeader::decode(const uint8_t* buffer) {
    MessageHeader header;
    header.id = buffer[0];
    header.len = (buffer[1] << 8) | buffer[2];  // big-endian unpack
    header.pad = buffer[3];  // always 0
    return header;
}

}  // namespace protocol
