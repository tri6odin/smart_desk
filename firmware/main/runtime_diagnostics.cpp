#include "runtime_diagnostics.hpp"

#include "desk_controller.hpp"
#include "desk_protocol.hpp"
#include "desk_recovery.hpp"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"

namespace runtime_diagnostics {
namespace {

constexpr std::uint32_t kMagic = 0x53444B31;  // SDK1
constexpr char kTag[] = "desk.diagnostics";

struct Snapshot {
    std::uint32_t magic;
    std::uint32_t sequence;
    std::uint32_t last_command;
    std::uint32_t control_state;
    std::uint32_t recovery_state;
    std::uint32_t height_tenths_cm;
    std::uint32_t last_fault;
    std::uint32_t checksum;
};

RTC_NOINIT_ATTR Snapshot g_snapshot;
portMUX_TYPE g_lock = portMUX_INITIALIZER_UNLOCKED;

std::uint32_t checksum(const Snapshot& value) {
    return value.magic ^ value.sequence ^ value.last_command ^
           value.control_state ^ value.recovery_state ^
           value.height_tenths_cm ^ value.last_fault ^ 0xA5C39E42;
}

void update_checksum() {
    g_snapshot.checksum = checksum(g_snapshot);
}

const char* fault_name(std::uint32_t value) {
    switch (static_cast<Fault>(value)) {
        case Fault::kNone: return "none";
        case Fault::kUnknownControllerFrame: return "unknown-controller-frame";
        case Fault::kUartTransmit: return "uart-transmit";
        case Fault::kWriterStalled: return "writer-stalled";
    }
    return "unknown";
}

}  // namespace

void begin_boot() {
    const auto reason = esp_reset_reason();
    if (g_snapshot.magic == kMagic &&
        g_snapshot.checksum == checksum(g_snapshot)) {
        ESP_LOGW(kTag,
                 "previous boot: reset=%u tx_seq=%lu command=%s state=%s "
                 "recovery=%s height=%lu.%lu cm fault=%s",
                 static_cast<unsigned>(reason),
                 static_cast<unsigned long>(g_snapshot.sequence),
                 smart_desk::command_name(
                     static_cast<std::uint8_t>(g_snapshot.last_command)),
                 smart_desk::control_state_name(
                     static_cast<smart_desk::ControlState>(
                         g_snapshot.control_state)),
                 smart_desk::recovery_state_name(
                     static_cast<smart_desk::RecoveryState>(
                         g_snapshot.recovery_state)),
                 static_cast<unsigned long>(g_snapshot.height_tenths_cm / 10),
                 static_cast<unsigned long>(g_snapshot.height_tenths_cm % 10),
                 fault_name(g_snapshot.last_fault));
    } else {
        ESP_LOGI(kTag, "boot reset=%u; no retained diagnostic snapshot",
                 static_cast<unsigned>(reason));
    }
    g_snapshot = Snapshot{kMagic, 0, 0, 0, 0, 0, 0, 0};
    update_checksum();
}

void command(std::uint32_t value, std::uint32_t control_state,
             std::uint32_t recovery_state, std::uint32_t height_tenths_cm) {
    portENTER_CRITICAL(&g_lock);
    ++g_snapshot.sequence;
    g_snapshot.last_command = value;
    g_snapshot.control_state = control_state;
    g_snapshot.recovery_state = recovery_state;
    g_snapshot.height_tenths_cm = height_tenths_cm;
    update_checksum();
    portEXIT_CRITICAL(&g_lock);
}

void fault(Fault value) {
    portENTER_CRITICAL(&g_lock);
    g_snapshot.last_fault = static_cast<std::uint32_t>(value);
    update_checksum();
    portEXIT_CRITICAL(&g_lock);
}

}  // namespace runtime_diagnostics
