#ifndef MESSAGE_HEADER_HPP
#define MESSAGE_HEADER_HPP
#include <cstdint>
#include <cstddef>
namespace protocol {


struct MessageHeader {
    uint8_t id;     // Message ID (1 byte)
    uint16_t len;   // Payload length (2 bytes)
    uint8_t pad = 0;// Always 0 (1 byte)

    static constexpr size_t SIZE = 4;

   
    void encode(uint8_t* buffer) const;

  
    static MessageHeader decode(const uint8_t* buffer);
};

}  // namespace protocol
#endif // MESSAGE_HEADER_HPP
