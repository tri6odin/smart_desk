#pragma once

#include "desk_controller.hpp"

namespace smart_desk {

struct TelemetrySnapshot {
    std::uint16_t height_tenths_cm = 0;
    bool fresh = false;
    std::uint32_t stable_frames = 0;
    std::uint32_t anomaly_sequence = 0;
    std::uint32_t height_frame_sequence = 0;
    std::uint32_t reset_frame_sequence = 0;
    std::uint32_t unknown_frame_sequence = 0;
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
    bool calibration_requested = false;
    std::uint32_t user_activity_sequence = 0;
};

// Edge-triggered detector for buttons whose HID report is sent once on press
// and once on release. Time is modulo 2^32, matching the controller clock.
class LongPressDetector {
   public:
    explicit LongPressDetector(std::uint32_t threshold) : threshold_(threshold) {}

    // Returns true only for the first report of a physical press.
    bool press(std::uint32_t now);
    bool poll(std::uint32_t now);
    bool release(std::uint32_t now);
    void reset();

   private:
    std::uint32_t threshold_;
    std::uint32_t pressed_at_ = 0;
    bool pressed_ = false;
    bool fired_ = false;
};

enum class PresetButton { kSit, kStand };

struct PresetPressResult {
    bool new_press = false;
    bool chord_started = false;
};

struct PresetReleaseResult {
    bool sit_pressed = false;
    bool stand_pressed = false;
    std::uint32_t sit_duration = 0;
    std::uint32_t stand_duration = 0;
    bool suppress_presets = false;
    bool start_calibration = false;
};

// Tracks SIT and STAND usages in the D100H Consumer Control report stream.
// The observed zero usage ends a gesture; reports do not carry a separate
// release event for each physical button.
class PresetGestureDetector {
   public:
    explicit PresetGestureDetector(std::uint32_t chord_threshold)
        : chord_(chord_threshold) {}

    PresetPressResult press(PresetButton button, std::uint32_t now);
    bool poll(std::uint32_t now);
    PresetReleaseResult release(std::uint32_t now);
    // Any other HID action invalidates an unfinished preset gesture. Keep it
    // suppressed until the zero-usage release so a stale button cannot move
    // the desk or restart the calibration timer.
    bool cancel();
    void reset();

   private:
    LongPressDetector chord_;
    std::uint32_t sit_pressed_at_ = 0;
    std::uint32_t stand_pressed_at_ = 0;
    bool sit_pressed_ = false;
    bool stand_pressed_ = false;
    bool chord_seen_ = false;
    bool cancelled_ = false;
};

// One replacement slot, no motion queue. All methods require the same lock
// when used from BLE callbacks and the writer task. STOP dominates every
// button received before the next consume(), regardless of arrival order.
class InputMailbox {
   public:
    void dial(int direction, std::uint32_t now_ms);
    void button(ButtonRequest request);
    void stop();
    void request_calibration();
    void user_activity();
    InputSnapshot consume();
    void set_controller_state(ControlState state, int direction) {
        state_ = state;
        active_direction_ = direction;
    }

   private:
    InputSnapshot pending_;
    ControlState state_ = ControlState::kIdle;
    int active_direction_ = 0;
};

}  // namespace smart_desk
