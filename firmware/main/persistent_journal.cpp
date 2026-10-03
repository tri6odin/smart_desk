#include "persistent_journal.hpp"

#include <atomic>
#include <cstring>

#include "esp_app_desc.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace persistent_journal {
namespace {
using namespace smart_desk::journal;
Capture g_capture;
portMUX_TYPE g_queue_lock = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t g_store_lock = nullptr;
std::atomic<bool> g_needs_sector{false};
bool g_enabled = false;

class PartitionFlash final : public Flash {
public:
    const esp_partition_t* partition = nullptr;
    std::size_t size() const override { return partition->size; }
    bool read(std::size_t at, void* data, std::size_t len) override {
        return esp_partition_read(partition, at, data, len) == ESP_OK;
    }
    bool write(std::size_t at, const void* data, std::size_t len) override {
        return esp_partition_write(partition, at, data, len) == ESP_OK;
    }
    bool erase_sector(std::size_t at) override {
        return esp_partition_erase_range(partition, at, kSectorSize) == ESP_OK;
    }
} g_flash;
Store g_store(g_flash);

std::uint64_t uptime_ms() {
    return static_cast<std::uint64_t>(esp_timer_get_time() / 1000);
}

bool take_record(Record& record) {
    portENTER_CRITICAL(&g_queue_lock);
    const auto next = g_capture.take(uptime_ms());
    portEXIT_CRITICAL(&g_queue_lock);
    if (next) record = *next;
    return next.has_value();
}

bool safety_cycle_pending() {
    portENTER_CRITICAL(&g_queue_lock);
    const bool pending = g_capture.safety_pending();
    portEXIT_CRITICAL(&g_queue_lock);
    return pending;
}

void worker(void*) {
    Record pending;
    bool have_pending = false;
    while (true) {
        if (!have_pending) {
            have_pending = take_record(pending);
            if (have_pending) seal(pending);
        }
        if (!have_pending || safety_cycle_pending() || g_needs_sector.load()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        xSemaphoreTake(g_store_lock, portMAX_DELAY);
        const auto result = g_store.append(pending);
        xSemaphoreGive(g_store_lock);
        if (result == AppendResult::kWritten) {
            have_pending = false;
            // Yield between writes even during a backlog.
            vTaskDelay(1);
        } else if (result == AppendResult::kNeedsSector) {
            g_needs_sector.store(true);
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
}  // namespace

void begin() {
    g_flash.partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, static_cast<esp_partition_subtype_t>(0x40), "desk_log");
    if (g_flash.partition == nullptr || !g_store.mount()) {
        return;
    }
    g_store_lock = xSemaphoreCreateMutex();
    if (g_store_lock == nullptr) {
        return;
    }
    g_enabled = true;
    smart_desk::Frame build{};
    std::memcpy(build.data(), esp_app_get_description()->app_elf_sha256, build.size());
    g_capture.begin(g_store.last_boot_id() + 1, g_store.last_sequence(), uptime_ms(),
                    build, static_cast<std::uint8_t>(esp_reset_reason()));
}

void start_worker() {
    if (g_enabled && xTaskCreate(worker, "desk_log", 4096, nullptr, 2, nullptr) != pdPASS) {
        g_enabled = false;
    }
}

void receive(const smart_desk::Frame& frame, bool invalid) {
    if (!g_enabled) return;
    portENTER_CRITICAL(&g_queue_lock);
    g_capture.receive(frame, uptime_ms(), invalid);
    portEXIT_CRITICAL(&g_queue_lock);
}

void transmitted(smart_desk::Command command, Context context, bool routine_step_idle,
                 StopReason reason) {
    if (!g_enabled) return;
    portENTER_CRITICAL(&g_queue_lock);
    g_capture.transmitted(command, uptime_ms(), context, routine_step_idle, reason);
    portEXIT_CRITICAL(&g_queue_lock);
}

void transmit_error(smart_desk::Command command, Context context, StopReason reason) {
    if (!g_enabled) return;
    portENTER_CRITICAL(&g_queue_lock);
    g_capture.transmit_error(command, uptime_ms(), context, reason);
    portEXIT_CRITICAL(&g_queue_lock);
}

void presets(std::uint16_t previous_sit, std::uint16_t previous_stand,
             std::uint16_t sit, std::uint16_t stand,
             smart_desk::journal::Context context) {
    if (!g_enabled) return;
    portENTER_CRITICAL(&g_queue_lock);
    g_capture.presets(previous_sit, previous_stand, sit, stand, uptime_ms(), context);
    portEXIT_CRITICAL(&g_queue_lock);
}

void cycle_completed(std::uint32_t unknown_sequence) {
    if (!g_enabled) return;
    portENTER_CRITICAL(&g_queue_lock);
    g_capture.cycle_completed(unknown_sequence);
    portEXIT_CRITICAL(&g_queue_lock);
}

void maintain_after_idle() {
    static std::int64_t retry_after_us = 0;  // Only accessed by the UART writer.
    if (!g_enabled || !g_needs_sector.load() ||
        esp_timer_get_time() < retry_after_us ||
        xSemaphoreTake(g_store_lock, 0) != pdTRUE) return;
    if (g_store.prepare_sector()) g_needs_sector.store(false);
    else {
        retry_after_us = esp_timer_get_time() + 10000000;
    }
    xSemaphoreGive(g_store_lock);
}

}  // namespace persistent_journal
