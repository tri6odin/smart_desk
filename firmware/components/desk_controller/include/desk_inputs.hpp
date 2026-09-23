#pragma once

#include "desk_controller.hpp"

namespace smart_desk {

struct TelemetrySnapshot {
    std::uint16_t height_tenths_cm = 0;
    bool fresh = false;
    std::uint32_t stable_frames = 0;
    std::uint32_t anomaly_sequence = 0;
};

// Pure state: the ESP adapter must serialize receive() and snapshot(). Time
// is monotonic milliseconds modulo 2^32, just like ControllerInput::now_ms.
class TelemetryTracker {
   public:
    static constexpr std::uint32_t kMaxAgeMs = 300;
    explicit TelemetryTracker(ControllerConfig config = {});
    void receive(const Frame& frame, std::uint32_t now_ms);
    TelemetrySnapshot snapshot(std::uint32_t now_ms) const;

   private:
    ControllerConfig config_;
    TelemetrySnapshot state_;
    std::uint32_t last_ms_ = 0;
    bool available_ = false;
};

enum class ButtonRequest { kNone, kGoSit, kGoStand, kSaveSit, kSaveStand };

struct InputSnapshot {
    std::uint32_t dial_sequence = 0;
    int dial_direction = 0;
    std::uint32_t last_dial_ms = 0;
    ButtonRequest button = ButtonRequest::kNone;
    bool stop_requested = false;
};

// One replacement slot, no motion queue. All methods require the same lock
// when used from BLE callbacks and the writer task. STOP dominates every
// button received before the next consume(), regardless of arrival order.
class InputMailbox {
   public:
    void dial(int direction, std::uint32_t now_ms);
    void button(ButtonRequest request);
    void stop();
    InputSnapshot consume();
    void set_active_direction(int direction) { active_direction_ = direction; }

   private:
    InputSnapshot pending_;
    int active_direction_ = 0;
};

}  // namespace smart_desk
