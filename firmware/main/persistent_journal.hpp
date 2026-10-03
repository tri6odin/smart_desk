#pragma once

#include "desk_journal.hpp"

namespace persistent_journal {

// Mount and queue a boot marker. No flash erase is performed here.
void begin();
void start_worker();
// These capture paths only touch a bounded RAM queue; no flash or printing.
void receive(const smart_desk::Frame& frame, bool invalid = false);
void transmitted(smart_desk::Command command,
                 smart_desk::journal::Context context = smart_desk::journal::Context::kNone,
                 bool routine_step_idle = false);
void transmit_error(smart_desk::Command command);
void presets(std::uint16_t previous_sit, std::uint16_t previous_stand,
             std::uint16_t sit, std::uint16_t stand,
             smart_desk::journal::Context context = smart_desk::journal::Context::kNone);
// Called after the writer has handled this telemetry snapshot, including
// attempting the safety IDLE. Newer unknown frames keep persistence gated.
void cycle_completed(std::uint32_t unknown_sequence);
// Only the UART writer calls this, AFTER successfully sending IDLE and when
// neither an autonomous STEP nor HOLD/calibration is active. At most one
// sector is erased; motion cannot begin in this task until it returns.
void maintain_after_idle();

}  // namespace persistent_journal
