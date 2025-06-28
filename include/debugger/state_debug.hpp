#ifndef STATE_DEBUG_HPP
#define STATE_DEBUG_HPP
#include <vector>
#include <cstdint>

namespace protocol{
    void debugPrintFrankaArmStateBuffer(const std::vector<uint8_t>&buffer);
}



#endif // STATE_DEBUG_HPP