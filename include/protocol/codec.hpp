#ifndef CODEC_HPP
#define CODEC_HPP
#include "protocol/byte_order.hpp"
#include "protocol/message_header.hpp"
#include "protocol/msg_id.hpp"
#include "protocol/franka_arm_state.hpp"
#include <cstdint>
#include <cstring>
#include <array>
#include <vector>

namespace protocol {

// encode std::array<double, N> (fixed size) 
template <size_t N>
inline void encode_array_f64(uint8_t*& ptr, const std::array<double, N>& in) {
    for (size_t i = 0; i < N; ++i) {
        double be_val = to_big_endian_f64(in[i]);
        std::memcpy(ptr, &be_val, sizeof(double));
        ptr += sizeof(double);
    }
}

// encoede std::vector<double> (dynamic size) 
inline void encode_array_f64(uint8_t*& ptr, const std::vector<double>& in) {
    for (const auto& val : in) {
        double be_val = to_big_endian_f64(val);
        std::memcpy(ptr, &be_val, sizeof(double));
        ptr += sizeof(double);
    }
}

// decode std::array<double, N> (fixed size)
template <size_t N>
inline void decode_array_f64(const uint8_t*& ptr, std::array<double, N>& out) {
    for (size_t i = 0; i < N; ++i) {
        double raw;
        std::memcpy(&raw, ptr, sizeof(double));
        out[i] = from_big_endian_f64(raw);
        ptr += sizeof(double);
    }
}

// decode std::vector<double>（dynamic size）
inline void decode_array_f64(const uint8_t*& ptr, std::vector<double>& out, size_t count) {
    out.resize(count);
    for (size_t i = 0; i < count; ++i) {
        double raw;
        std::memcpy(&raw, ptr, sizeof(double));
        out[i] = from_big_endian_f64(raw);
        ptr += sizeof(double);
    }
}

// encode uint32_t
inline void encode_u32(uint8_t*& ptr, uint32_t val) {
    uint32_t be_val = to_big_endian_u32(val);
    std::memcpy(ptr, &be_val, sizeof(be_val));
    ptr += sizeof(be_val);
}

// decode uint32_t
inline uint32_t decode_u32(const uint8_t*& ptr) {
    uint32_t raw;
    std::memcpy(&raw, ptr, sizeof(raw));
    ptr += sizeof(raw);
    return from_big_endian_u32(raw);
}

std::vector<uint8_t> encodeMessage(const MessageHeader& header, const std::vector<uint8_t>& payload);
std::vector<uint8_t> encodeStateMessage(const FrankaArmState& state);
std::vector<uint8_t> encodeModeMessage(uint8_t mode_code);
std::vector<uint8_t> encodeErrorMessage(uint8_t error_code);




}  // namespace protocol
#endif // CODEC_HPP