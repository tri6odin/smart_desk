#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "desk_protocol.hpp"

namespace smart_desk::journal {

constexpr std::size_t kSectorSize = 4096;
constexpr std::uint32_t kMagic = 0x314A4453;  // SDJ1, little endian
constexpr std::uint16_t kVersion = 1;

enum class Kind : std::uint8_t {
    kBoot = 1, kRx, kTx, kRxInvalid, kTxError, kGap, kPresets,
};
enum class Context : std::uint8_t {
    kNone = 0, kDial, kSit, kStand, kCalibration,
};

// Fixed on-flash format. All integer fields are little endian on ESP32-S3.
// CRC-32/ISO-HDLC covers bytes [0, 28); no pointers or dynamic strings.
struct Record {
    std::uint64_t uptime_ms = 0;
    std::uint32_t boot_id = 0;
    std::uint32_t sequence = 0;
    Frame data{};
    Kind kind{};
    std::uint8_t context = 0;
    std::uint16_t reserved = 0;
    std::uint32_t crc = 0;
};

struct SectorHeader {
    std::uint32_t magic = kMagic;
    std::uint16_t version = kVersion;
    std::uint16_t record_size = sizeof(Record);
    std::uint64_t generation = 0;
    std::uint32_t next_sequence = 1;
    std::uint32_t boot_id = 0;
    std::uint32_t reserved = 0;
    std::uint32_t crc = 0;
};
static_assert(sizeof(Record) == 32 && offsetof(Record, crc) == 28);
static_assert(sizeof(SectorHeader) == 32 && offsetof(SectorHeader, crc) == 28);
constexpr std::size_t kRecordsPerSector =
    (kSectorSize - sizeof(SectorHeader)) / sizeof(Record);

std::uint32_t crc32(const void* bytes, std::size_t size);
bool valid(const Record& record);
bool valid(const SectorHeader& header);
void seal(Record& record);

// Only the actual previous value in each direction participates in dedup.
// A suppressed STEP -> IDLE still updates the TX baseline.
class Filter {
public:
    bool receive(const Frame& frame, bool invalid = false);
    bool transmit(Command command, bool routine_step_idle = false);
    bool transmit_error(Command command);
private:
    Frame last_rx_{};
    bool have_rx_ = false;
    bool last_rx_invalid_ = false;
    Command last_tx_ = Command::kIdle;
    bool have_tx_ = false;
    bool last_tx_error_ = false;
};

// Pure capture/queue state. The adapter supplies uptime and serialization;
// no allocation, formatting, flash I/O or CRC in the capture path.
class Capture {
public:
    static constexpr std::size_t kCapacity = 1024;
    void begin(std::uint32_t boot, std::uint32_t previous_sequence,
               std::uint64_t now, const Frame& build, std::uint8_t reset_reason);
    void receive(const Frame& frame, std::uint64_t now, bool invalid = false);
    void transmitted(Command command, std::uint64_t now, Context context,
                     bool routine_step_idle);
    void transmit_error(Command command, std::uint64_t now);
    // Payload: previous SIT/STAND, new SIT/STAND (four little-endian u16).
    // Context none records the initial NVS snapshot. Saves log only changes.
    void presets(std::uint16_t previous_sit, std::uint16_t previous_stand,
                 std::uint16_t sit, std::uint16_t stand, std::uint64_t now,
                 Context context = Context::kNone);
    void cycle_completed(std::uint32_t unknown_sequence) { handled_ = unknown_sequence; }
    bool safety_pending() const { return unknown_ != handled_; }
    std::optional<Record> take(std::uint64_t now);
private:
    Record make_record(Kind kind, const Frame& data, std::uint64_t now,
                       std::uint8_t context = 0);
    void enqueue(Kind kind, const Frame& data, std::uint64_t now,
                 std::uint8_t context = 0);
    std::array<Record, kCapacity> queue_{};
    std::size_t head_ = 0, count_ = 0;
    std::uint64_t dropped_ = 0;
    std::uint32_t boot_ = 0, sequence_ = 0, unknown_ = 0, handled_ = 0;
    Filter filter_;
};

class Flash {
public:
    virtual ~Flash() = default;
    virtual std::size_t size() const = 0;
    virtual bool read(std::size_t offset, void* data, std::size_t size) = 0;
    virtual bool write(std::size_t offset, const void* data, std::size_t size) = 0;
    virtual bool erase_sector(std::size_t offset) = 0;
};

enum class AppendResult { kWritten, kNeedsSector, kIoError };

// Single-owner storage state; adapters must serialize access. Append NEVER
// erases. prepare_sector() is called only at startup or after a safe IDLE.
class Store {
public:
    explicit Store(Flash& flash) : flash_(flash) {}
    bool mount();
    bool prepare_sector();
    AppendResult append(const Record& record);
    std::uint32_t last_boot_id() const { return boot_id_; }
    std::uint32_t last_sequence() const { return sequence_; }
    std::uint64_t generation() const { return generation_; }
    bool needs_sector() const { return slot_ == kRecordsPerSector; }
private:
    bool open_next(bool erase);
    Flash& flash_;
    std::size_t sector_ = 0;
    std::size_t slot_ = kRecordsPerSector;
    std::uint64_t generation_ = 0;
    std::uint32_t sequence_ = 0;
    std::uint32_t boot_id_ = 0;
};

}  // namespace smart_desk::journal
