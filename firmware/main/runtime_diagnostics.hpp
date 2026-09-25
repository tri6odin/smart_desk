#pragma once

#include <cstdint>

namespace runtime_diagnostics {

enum class Fault : std::uint32_t {
    kNone = 0,
    kUnknownControllerFrame = 1,
    kUartTransmit = 2,
    kWriterStalled = 3,
};

void begin_boot();
void command(std::uint32_t value, std::uint32_t control_state,
             std::uint32_t recovery_state, std::uint32_t height_tenths_cm);
void fault(Fault value);

}  // namespace runtime_diagnostics
